#![feature(asm)]
#![feature(global_asm)]
#![feature(naked_functions)]

const DEFAULT_STACK_SIZE: usize = 1024 * 1024 * 2;

pub struct Runtime {
  threads: Vec<Thread>,
  current: usize,
}

#[repr(transparent)]
pub struct RuntimeRef(*mut Runtime);

#[derive(PartialEq, Eq, Debug)]
enum State {
  Available,
  Running,
  Ready,
}

struct Thread {
  stack: Box<[u8; DEFAULT_STACK_SIZE]>,
  ctx: ThreadContext,
  state: State,
}

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
  fn new() -> Self {
    Thread {
      stack: Box::new([0_u8; DEFAULT_STACK_SIZE]),
      ctx: ThreadContext::default(),
      state: State::Available,
    }
  }
}

impl Runtime {
  pub fn new() -> Self {
    Runtime {
      threads: vec![Thread {
        stack: Box::new([0_u8; DEFAULT_STACK_SIZE]),
        ctx: ThreadContext::default(),
        state: State::Running,
      }],
      current: 0,
    }
  }

  pub fn run(&mut self) {
    unsafe {
      while self.do_yield() {}
    }
  }

  unsafe fn do_yield(&mut self) -> bool {
    let old = self.current;
    if let Some(mut pos) = self.threads.iter()
      .skip(old)
      .chain(self.threads.iter().take(old))
      .position(|t| t.state == State::Ready) {
      pos += old;
      if pos >= self.threads.len() {
        pos -= self.threads.len();
      }
      if self.threads.get_unchecked_mut(old).state == State::Running {
        self.threads.get_unchecked_mut(old).state = State::Ready;
      }
      self.threads.get_unchecked_mut(pos).state = State::Running;
      self.current = pos;
      switch(&mut self.threads.get_unchecked_mut(old).ctx,
             &self.threads.get_unchecked_mut(pos).ctx, self);
      true
    } else { false }
  }

  pub fn spawn(&mut self, f: fn(RuntimeRef)) {
    let self_addr = self as *mut Runtime as u64;
    let available = if let Some(available) = self.threads.iter_mut()
      .find(|t| t.state == State::Available) {
      available
    } else {
      self.threads.push(Thread::new());
      self.threads.last_mut().unwrap()
    };

    unsafe {
      let s_ptr = available.stack.as_mut_ptr().add(available.stack.len());
      let s_ptr = (s_ptr as usize & !15) as *mut u8;
      (s_ptr as *mut u64).write(self_addr);
      (s_ptr.offset(-8) as *mut u64).write(guard as u64);
      (s_ptr.offset(-16) as *mut u64).write(f as u64);
      available.ctx.rsp = s_ptr.offset(-16) as u64;
    }
    available.state = State::Ready;
  }
}

impl RuntimeRef {
  pub fn do_yield(&self) {
    unsafe { (*self.0).do_yield(); }
  }
}

#[naked]
unsafe fn guard() {
  let rt: *mut Runtime;
  asm!("mov 0x0(%rsp), $0": "=r"(rt));
  let rt = &mut *rt;
  rt.threads.get_unchecked_mut(rt.current).state = State::Available;
  rt.do_yield();
  std::hint::unreachable_unchecked();
}

global_asm!("
  .global switch
switch:
  mov %rsp, 0x00(%rdi)
  mov %r15, 0x08(%rdi)
  mov %r14, 0x10(%rdi)
  mov %r13, 0x18(%rdi)
  mov %r12, 0x20(%rdi)
  mov %rbx, 0x28(%rdi)
  mov %rbp, 0x30(%rdi)

  mov %rdx, %rdi
  mov 0x00(%rsi), %rsp
  mov 0x08(%rsi), %r15
  mov 0x10(%rsi), %r14
  mov 0x18(%rsi), %r13
  mov 0x20(%rsi), %r12
  mov 0x28(%rsi), %rbx
  mov 0x30(%rsi), %rbp
  ret
");

extern "C" {
  #[allow(improper_ctypes)]
  fn switch(old: *mut ThreadContext, new: *const ThreadContext, rt: *mut Runtime);
}

fn main() {
  let mut runtime = Runtime::new();
  runtime.spawn(|rt| {
    println!("THREAD 1 STARTING");
    let id = 1;
    for i in 0..10 {
      println!("thread: {} counter: {}", id, i);
      rt.do_yield();
    }
    println!("THREAD 1 FINISHED");
  });
  runtime.spawn(|rt| {
    println!("THREAD 2 STARTING");
    let id = 2;
    for i in 0..15 {
      println!("thread: {} counter: {}", id, i);
      rt.do_yield();
    }
    println!("THREAD 2 FINISHED");
  });
  runtime.run();
}
