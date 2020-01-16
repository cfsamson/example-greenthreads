I have only tested my implementation on Linux.

I have done the following modifications mainly:

# Some small modifications

Some of these modifications are only for my personal preference.

1. Rename:
    - `t_yield` -> `do_yield`
    - `t_return` -> `do_return`
2. Change index expressions to `get_unchecked`.
3. Change indent, and compress some lines. Now it is only 176 lines long.
4. Remove the `id` field in `Thread`, since it is not used.
5. Change the type of `stack` from `Vec<u8>` to `Box<[u8; DEFAULT_STACK_SIZE]>`, so that we won't need to worry about the underlying pointer is moved.
6. Remove the limitation of `MAX_THREADS`. Now I don't allocate extra threads when initializing `Runtime`, and only a base thread is created. When user calls `spawn` and their is no available thread, I will allocate one more thread and push it to `threads`.

# Remove global variables

Originally the code use mutable global variable `RUNTIME` to track the address of the `Runtime` instance, which is not favored in Rust, and it disables user to have multiple `Runtime` instances, or move a `Runtime` instance.

I managed to remove this global variable. In order to do this, we need to be able to access the `Runtime` instance in function `guard`, which is called when user-defined thread function ends, and `do_yield`, which is called by user-defined thread function. 

For the first usage, we can store the address of the `Runtime` instance in the thread stack, and let `guard` fetch it out, and then can invoke `do_return` on it.

Related code:

```rust
impl RunTime {
  ...
  unsafe fn do_yield(&mut self) {
    ...
    // I don't fully understand why the original code use offset -24 and -32
    // it seems a waste of space, am I right?
    (s_ptr as *mut u64).write(self_addr); // `guard` can access it
    (s_ptr.offset(-8) as *mut u64).write(guard as u64);
    (s_ptr.offset(-16) as *mut u64).write(f as u64);
    available.ctx.rsp = s_ptr.offset(-16) as u64;
    ...
  }
  ...
}

#[naked] // it need to be naked, otherwise %rsp may not point to `self_addr` when executing the asm! block
unsafe fn guard() {
  let rt: *mut Runtime;
  asm!("mov 0x0(%rsp), $0": "=r"(rt)); // fetch `self_addr` out
  let rt = &mut *rt;
  // the original content of do_return, copied to here
  // the `if current != 0` logic is removed, I think `current` can never be 0 here, am I right?
  rt.threads.get_unchecked_mut(rt.current).state = State::Available;
  rt.do_yield();
  // their must exist a Ready thread to execute
  // and this thread will not be executed before a new task has arrived, and the context is overwritten
  // so `do_yield` will never return
  std::hint::unreachable_unchecked();
}
```

For the second usage, we can add an extra parameter to the user-defined thread function. I defines a struct `RuntimeRef`, which holds a pointer to the `Runtime` instance. `RuntimeRef` only exposed the `do_yield` interface to user, so that user cannot do other modification to the `Runtime` instance (maybe calling `spawn`? I am not sure whether we can handle this properly).

Related code:

```rust
// I found that if `switch` is declared as a naked function, the code won't work in debug mode
// because compiler will write to and read from the stack to satisfy the asm template constraint
// (of course it is unnecessary, but the compiler generate such code in debug mode anyway)
// this function being naked, the stack pointer is not adjusted, so writing to the stack will modify the return address on the stack
// to prevent compiler from doing anything strange, I write `switch` completely in asm

global_asm!("
  .global switch
switch:
  mov %rsp, 0x00(%rdi)
  ...
  mov %rbp, 0x30(%rdi)

  mov %rdx, %rdi # pass the 3rd arg of `switch` to the 1st arg of user-defined thread function
  mov 0x00(%rsi), %rsp
  ...
  mov 0x30(%rsi), %rbp
  ret
");

extern "C" {
  // without this attribute, compiler will warn about the 3rd arg
  // saying that `Runtime` is not suitable for ffi interface
  // but we won't access the fields in `Runtime`, but just use it as an opaque pointer, so it is okay
  #[allow(improper_ctypes)]
  fn switch(old: *mut ThreadContext, new: *const ThreadContext, rt: *mut Runtime);
}
```

Now user can use `Thread::spawn` in this way:

```rust
runtime.spawn(|rt| {
  println!("THREAD 1 STARTING");
  let id = 1;
  for i in 0..10 {
    println!("thread: {} counter: {}", id, i);
    rt.do_yield();
  }
  println!("THREAD 1 FINISHED");
});
``` 