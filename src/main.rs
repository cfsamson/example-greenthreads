#![feature(llvm_asm)]
#![feature(naked_functions)]
use std::ptr;

// In our simple example we set most constraints here.
const DEFAULT_STACK_SIZE: usize = 1024 * 1024 * 2;
const MAX_THREADS: usize = 4;
static mut RUNTIME: usize = 0;

pub struct Runtime {
    threads: Vec<Thread>,
    current: usize,
}

#[derive(PartialEq, Eq, Debug)]
enum State {
    Available,
    Running,
    Ready,
}

struct Thread {
    id: usize,
    stack: Vec<u8>,
    ctx: ThreadContext,
    state: State,
}

#[derive(Debug, Default)]
#[repr(C)] // not strictly needed but Rust ABI is not guaranteed to be stable
struct ThreadContext {
    rsp: u64,
    r15: u64,
    r14: u64,
    r13: u64,
    r12: u64,
    rbx: u64,
    rbp: u64,
}

impl Thread {
    fn new(id: usize) -> Self {
        // We initialize each thread here and allocate the stack. This is not neccesary,
        // we can allocate memory for it later, but it keeps complexity down and lets us focus on more interesting parts
        // to do it here. The important part is that once allocated it MUST NOT move in memory.
        Thread {
            id,
            stack: vec![0_u8; DEFAULT_STACK_SIZE],
            ctx: ThreadContext::default(),
            state: State::Available,
        }
    }
}

impl Runtime {
    pub fn new() -> Self {
        // This will be our base thread, which will be initialized in the `running` state
        let base_thread = Thread {
            id: 0,
            stack: vec![0_u8; DEFAULT_STACK_SIZE],
            ctx: ThreadContext::default(),
            state: State::Running,
        };

        // We initialize the rest of our threads.
        let mut threads = vec![base_thread];
        let mut available_threads: Vec<Thread> = (1..MAX_THREADS).map(|i| Thread::new(i)).collect();
        threads.append(&mut available_threads);

        Runtime {
            threads,
            current: 0,
        }
    }

    /// This is cheating a bit, but we need a pointer to our Runtime stored so we can call yield on it even if
    /// we don't have a reference to it.
    pub fn init(&self) {
        unsafe {
            let r_ptr: *const Runtime = self;
            RUNTIME = r_ptr as usize;
        }
    }

    /// This is where we start running our runtime. If it is our base thread, we call yield until
    /// it returns false (which means that there are no threads scheduled) and we are done.
    pub fn run(&mut self) -> ! {
        while self.t_yield() {}
        std::process::exit(0);
    }

    /// This is our return function. The only place we use this is in our `guard` function.
    /// If the current thread is not our base thread we set its state to Available. It means
    /// we're finished with it. Then we yield which will schedule a new thread to be run.
    fn t_return(&mut self) {
        if self.current != 0 {
            self.threads[self.current].state = State::Available;
            self.t_yield();
        }
    }

    /// This is the heart of our runtime. Here we go through all threads and see if anyone is in the `Ready` state.
    /// If no thread is `Ready` we're all done. This is an extremely simple sceduler using only a round-robin algorithm.
    ///
    /// If we find a thread that's ready to be run we change the state of the current thread from `Running` to `Ready`.
    /// Then we call switch which will save the current context (the old context) and load the new context
    /// into the CPU which then resumes based on the context it was just passed.
    fn t_yield(&mut self) -> bool {
        let mut pos = self.current;
        while self.threads[pos].state != State::Ready {
            pos += 1;

            if pos == self.threads.len() {
                pos = 0;
            }
            if pos == self.current {
                return false;
            }
        }

        if self.threads[self.current].state != State::Available {
            self.threads[self.current].state = State::Ready;
        }

        self.threads[pos].state = State::Running;
        let old_pos = self.current;
        self.current = pos;

        unsafe {
            switch(&mut self.threads[old_pos].ctx, &self.threads[pos].ctx);
        }

        // NOTE: this might look strange and it is. Normally we would just mark this as `unreachable!()` but our compiler
        // is too smart for it's own good so it optimized our code away on release builds. Curiously this happens on windows
        // and not on linux. This is a common problem in tests so Rust has a `black_box` function in the `test` crate that
        // will "pretend" to use a value we give it to prevent the compiler from eliminating code. I'll just do this instead,
        // this code will never be run anyways and if it did it would always be `true`.
        self.threads.len() > 0
    }

    /// While `yield` is the logically interesting function I think this the technically most interesting.
    ///
    ///
    /// When we spawn a new thread we first check if there are any available threads (threads in `Parked` state).
    /// If we run out of threads we panic in this scenario but there are several (better) ways to handle that.
    /// We keep things simple for now.
    ///
    /// When we find an available thread we get the stack length and a pointer to our u8 bytearray.
    ///
    /// The next part we have to use some unsafe functions. First we write an address to our `guard` function
    /// that will be called if the function we provide returns. Then we set the address to the function we
    /// pass inn.
    ///
    /// Third, we set the value of `rsp` which is the stack pointer to the address of our provided function so we start
    /// executing that first when we are scheuled to run.
    ///
    /// Lastly we set the state as `Ready` which means we have work to do and is ready to do it.
    pub fn spawn(&mut self, f: fn()) {
        let available = self
            .threads
            .iter_mut()
            .find(|t| t.state == State::Available)
            .expect("no available thread.");

        let size = available.stack.len();

        unsafe {
            let s_ptr = available.stack.as_mut_ptr().offset(size as isize);
            
            // make sure our stack itself is 16 byte aligned - it will always
            // offset to a lower memory address. Since we know we're at the "high"
            // memory address of our allocated space, we know that offsetting to
            // a lower one will be a valid address (given that we actually allocated)
            // enough space to actually get an aligned pointer in the first place).
            let s_ptr = (s_ptr as usize & ! 15) as *mut u8;
            ptr::write(s_ptr.offset(-24) as *mut u64, guard as u64);
            ptr::write(s_ptr.offset(-32) as *mut u64, f as u64);
            available.ctx.rsp = s_ptr.offset(-32) as u64;
        }

        available.state = State::Ready;
    }
}

/// This is our guard function that we place on top of the stack. All this function does is set the
/// state of our current thread and then `yield` which will then schedule a new thread to be run.
fn guard() {
    unsafe {
        let rt_ptr = RUNTIME as *mut Runtime;
        (*rt_ptr).t_return();
    };
}

/// We know that Runtime is alive the length of the program and that we only access from one core
/// (so no datarace). We yield execution of the current thread  by dereferencing a pointer to our
/// Runtime and then calling `t_yield`
pub fn yield_thread() {
    unsafe {
        let rt_ptr = RUNTIME as *mut Runtime;
        (*rt_ptr).t_yield();
    };
}

/// So here is our inline Assembly. As you remember from our first example this is just a bit more elaborate where we first
/// read out the values of all the registers we need and then sets all the register values to the register values we
/// saved when we suspended exceution on the "new" thread.
///
/// This is essentially all we need to do to save and resume execution.
///
/// Some details about inline assembly.
///
/// The assembly commands in the string literal is called the assemblt template. It is preceeded by
/// zero or up to four segments indicated by ":":
///
/// - First ":" we have our output parameters, this parameters that this function will return.
/// - Second ":" we have the input parameters which is our contexts. We only read from the "new" context
/// but we modify the "old" context saving our registers there (see volatile option below)
/// - Third ":" This our clobber list, this is information to the compiler that these registers can't be used freely
/// - Fourth ":" This is options we can pass inn, Rust has 3: "alignstack", "volatile" and "intel"
///
/// For this to work on windows we need to use "alignstack" where the compiler adds the neccesary padding to
/// make sure our stack is aligned. Since we modify one of our inputs, our assembly has "side effects"
/// therefore we should use the `volatile` option. I **think** this is actually set for us by default
/// when there are no output parameters given (my own assumption after going through the source code)
/// for the `asm` macro, but we should make it explicit anyway.
///
/// One last important part (it will not work without this) is the #[naked] attribute. Basically this lets us have full
/// control over the stack layout since normal functions has a prologue-and epilogue added by the
/// compiler that will cause trouble for us. We avoid this by marking the funtion as "Naked".
/// For this to work on `release` builds we also need to use the `#[inline(never)] attribute or else
/// the compiler decides to inline this function (curiously this currently only happens on Windows).
/// If the function is inlined we get a curious runtime error where it fails when switching back
/// to as saved context and in general our assembly will not work as expected.
///
/// see: https://github.com/rust-lang/rfcs/blob/master/text/1201-naked-fns.md
#[naked]
#[inline(never)]
unsafe fn switch(old: *mut ThreadContext, new: *const ThreadContext) {
    llvm_asm!("
        mov     %rsp, 0x00($0)
        mov     %r15, 0x08($0)
        mov     %r14, 0x10($0)
        mov     %r13, 0x18($0)
        mov     %r12, 0x20($0)
        mov     %rbx, 0x28($0)
        mov     %rbp, 0x30($0)
   
        mov     0x00($1), %rsp
        mov     0x08($1), %r15
        mov     0x10($1), %r14
        mov     0x18($1), %r13
        mov     0x20($1), %r12
        mov     0x28($1), %rbx
        mov     0x30($1), %rbp
        ret
        "
    :
    :"r"(old), "r"(new)
    :
    : "volatile", "alignstack"
    );
}

fn main() {
    let mut runtime = Runtime::new();
    runtime.init();

    runtime.spawn(|| {
        println!("THREAD 1 STARTING");
        let id = 1;
        for i in 0..10 {
            println!("thread: {} counter: {}", id, i);
            yield_thread();
        }

        println!("THREAD 1 FINISHED");
    });

    runtime.spawn(|| {
        println!("THREAD 2 STARTING");
        let id = 2;
        for i in 0..15 {
            println!("thread: {} counter: {}", id, i);
            yield_thread();
        }

        println!("THREAD 2 FINISHED");
    });

    runtime.run();
}
