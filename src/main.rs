#![feature(llvm_asm, naked_functions)]

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

#[cfg(not(target_os = "windows"))]
#[derive(Debug, Default)]
#[repr(C)]
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
        let base_thread = Thread {
            id: 0,
            stack: vec![0_u8; DEFAULT_STACK_SIZE],
            ctx: ThreadContext::default(),
            state: State::Running,
        };

        let mut threads = vec![base_thread];
        let mut available_threads: Vec<Thread> = (1..MAX_THREADS).map(|i| Thread::new(i)).collect();
        threads.append(&mut available_threads);

        Runtime {
            threads,
            current: 0,
        }
    }

    pub fn init(&self) {
        unsafe {
            let r_ptr: *const Runtime = self;
            RUNTIME = r_ptr as usize;
        }
    }

    pub fn run(&mut self) -> ! {
        while self.t_yield() {}
        std::process::exit(0);
    }

    fn t_return(&mut self) {
        if self.current != 0 {
            self.threads[self.current].state = State::Available;
            self.t_yield();
        }
    }

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
            let old: *mut ThreadContext = &mut self.threads[old_pos].ctx;
            let new: *const ThreadContext = &self.threads[pos].ctx;

            if cfg!(not(target_os = "windows")) {
                llvm_asm!(
                    "mov $0, %rdi
                     mov $1, %rsi" ::"r"(old), "r"(new)
                );
            } else {
                llvm_asm!(
                    "mov $0, %rcx
                     mov $1, %rdx" ::"r"(old), "r"(new)
                );
            }

            switch();
        }

        // preventing compiler optimizing our code away on windows. Will never be reached anyway.
        self.threads.len() > 0
    }

    #[cfg(not(target_os = "windows"))]
    pub fn spawn(&mut self, f: fn()) {
        let available = self
            .threads
            .iter_mut()
            .find(|t| t.state == State::Available)
            .expect("no available thread.");

        let size = available.stack.len();
        let s_ptr = available.stack.as_mut_ptr();
        unsafe {
            std::ptr::write(s_ptr.offset((size - 16) as isize) as *mut u64, guard as u64);
            std::ptr::write(s_ptr.offset((size - 24) as isize) as *mut u64, skip as u64);
            std::ptr::write(s_ptr.offset((size - 32) as isize) as *mut u64, f as u64);
            available.ctx.rsp = s_ptr.offset((size - 32) as isize) as u64;
        }
        available.state = State::Ready;
    }
}

#[naked]
fn skip() {}

fn guard() {
    unsafe {
        let rt_ptr = RUNTIME as *mut Runtime;
        (*rt_ptr).t_return();
    };
}

pub fn yield_thread() {
    unsafe {
        let rt_ptr = RUNTIME as *mut Runtime;
        (*rt_ptr).t_yield();
    };
}

#[cfg(not(target_os = "windows"))]
#[naked]
#[inline(never)]
#[no_mangle]
unsafe fn switch() {
    llvm_asm!(
        "
        mov     %rsp, 0x00(%rdi)
        mov     %r15, 0x08(%rdi)
        mov     %r14, 0x10(%rdi)
        mov     %r13, 0x18(%rdi)
        mov     %r12, 0x20(%rdi)
        mov     %rbx, 0x28(%rdi)
        mov     %rbp, 0x30(%rdi)

        mov     0x00(%rsi), %rsp
        mov     0x08(%rsi), %r15
        mov     0x10(%rsi), %r14
        mov     0x18(%rsi), %r13
        mov     0x20(%rsi), %r12
        mov     0x28(%rsi), %rbx
        mov     0x30(%rsi), %rbp
        ret
        "
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

// ===== WINDOWS SUPPORT =====
#[cfg(target_os = "windows")]
#[derive(Debug, Default)]
#[repr(C)]
struct ThreadContext {
    xmm6: [u64; 2],
    xmm7: [u64; 2],
    xmm8: [u64; 2],
    xmm9: [u64; 2],
    xmm10: [u64; 2],
    xmm11: [u64; 2],
    xmm12: [u64; 2],
    xmm13: [u64; 2],
    xmm14: [u64; 2],
    xmm15: [u64; 2],
    rsp: u64,
    r15: u64,
    r14: u64,
    r13: u64,
    r12: u64,
    rbx: u64,
    rbp: u64,
    rdi: u64,
    rsi: u64,
    stack_start: u64,
    stack_end: u64,
}

impl Runtime {
    #[cfg(target_os = "windows")]
    pub fn spawn(&mut self, f: fn()) {
        let available = self
            .threads
            .iter_mut()
            .find(|t| t.state == State::Available)
            .expect("no available thread.");

        let size = available.stack.len();
        let s_ptr = available.stack.as_mut_ptr();

        // see: https://docs.microsoft.com/en-us/cpp/build/stack-usage?view=vs-2019#stack-allocation
        unsafe {
            std::ptr::write(s_ptr.offset((size - 16) as isize) as *mut u64, guard as u64);
            std::ptr::write(s_ptr.offset((size - 24) as isize) as *mut u64, skip as u64);
            std::ptr::write(s_ptr.offset((size - 32) as isize) as *mut u64, f as u64);
            available.ctx.rsp = s_ptr.offset((size - 32) as isize) as u64;
            available.ctx.stack_start = s_ptr.offset(size as isize) as u64;
        }
        available.ctx.stack_end = s_ptr as *const u64 as u64;

        available.state = State::Ready;
    }
}

// reference: https://probablydance.com/2013/02/20/handmade-coroutines-for-windows/
// Contents of TIB on Windows: https://en.wikipedia.org/wiki/Win32_Thread_Information_Block
#[cfg(target_os = "windows")]
#[inline(never)]
#[no_mangle]
unsafe fn switch() {
    llvm_asm!(
        "
        movaps      %xmm6, 0x00(%rcx)
        movaps      %xmm7, 0x10(%rcx)
        movaps      %xmm8, 0x20(%rcx)
        movaps      %xmm9, 0x30(%rcx)
        movaps      %xmm10, 0x40(%rcx)
        movaps      %xmm11, 0x50(%rcx)
        movaps      %xmm12, 0x60(%rcx)
        movaps      %xmm13, 0x70(%rcx)
        movaps      %xmm14, 0x80(%rcx)
        movaps      %xmm15, 0x90(%rcx)
        mov         %rsp, 0xa0(%rcx)
        mov         %r15, 0xa8(%rcx)
        mov         %r14, 0xb0(%rcx)
        mov         %r13, 0xb8(%rcx)
        mov         %r12, 0xc0(%rcx)
        mov         %rbx, 0xc8(%rcx)
        mov         %rbp, 0xd0(%rcx)
        mov         %rdi, 0xd8(%rcx)
        mov         %rsi, 0xe0(%rcx)
        mov         %gs:0x08, %rax
        mov         %rax, 0xe8(%rcx)
        mov         %gs:0x10, %rax
        mov         %rax, 0xf0(%rcx)

        movaps      0x00(%rdx), %xmm6
        movaps      0x10(%rdx), %xmm7
        movaps      0x20(%rdx), %xmm8
        movaps      0x30(%rdx), %xmm9
        movaps      0x40(%rdx), %xmm10
        movaps      0x50(%rdx), %xmm11
        movaps      0x60(%rdx), %xmm12
        movaps      0x70(%rdx), %xmm13
        movaps      0x80(%rdx), %xmm14
        movaps      0x90(%rdx), %xmm15
        mov         0xa0(%rdx), %rsp
        mov         0xa8(%rdx), %r15
        mov         0xb0(%rdx), %r14
        mov         0xb8(%rdx), %r13
        mov         0xc0(%rdx), %r12
        mov         0xc8(%rdx), %rbx
        mov         0xd0(%rdx), %rbp
        mov         0xd8(%rdx), %rdi
        mov         0xe0(%rdx), %rsi
        mov         0xe8(%rdx), %rax
        mov         %rax, %gs:0x08
        mov         0xf0(%rdx), %rax
        mov         %rax, %gs:0x10

        ret
        "
    );
}
