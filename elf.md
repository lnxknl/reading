0000000000001020 <_start>:
    1020:       f3 0f 1e fa             endbr64
    1024:       31 ed                   xor    %ebp,%ebp
    1026:       49 89 d1                mov    %rdx,%r9
    1029:       5e                      pop    %rsi
    102a:       48 89 e2                mov    %rsp,%rdx
    102d:       48 83 e4 f0             and    $0xfffffffffffffff0,%rsp
    1031:       50                      push   %rax
    1032:       54                      push   %rsp
    1033:       45 31 c0                xor    %r8d,%r8d
    1036:       31 c9                   xor    %ecx,%ecx
    1038:       48 8d 3d f0 00 00 00    lea    0xf0(%rip),%rdi        # 112f <main>
    103f:       ff 15 93 2f 00 00       call   *0x2f93(%rip)        # 3fd8 <__libc_start_main@GLIBC_2.34>
    1045:       f4                      hlt
    1046:       66 2e 0f 1f 84 00 00    cs nopw 0x0(%rax,%rax,1)
    104d:       00 00 00


[tong@free ~/work/test]$ ./a0
140732362591036
140732362591040
[tong@free ~/work/test]$ ./a0
140724541193180
140724541193184
[tong@free ~/work/test]$ vi a0.c
[tong@free ~/work/test]$ gcc -g -o a0 a0.c
a0.c: In function ‘foo’:
a0.c:7:17: warning: returning ‘int’ from a function with return type ‘int *’ makes pointer from integer without a cast [-Wint-conversion]
    7 |         return a+b;
      |                ~^~
a0.c: In function ‘main’:
a0.c:20:16: warning: initialization of ‘int’ from ‘int *’ makes integer from pointer without a cast [-Wint-conversion]
   20 |         int c= foo(a,b);
      |                ^~~
[tong@free ~/work/test]$ ./a0
140735444976844
140735444976848
94882615111744
[tong@free ~/work/test]$ ./a0
140725731460972
140725731460976
94074715385920
[tong@free ~/work/test]$ ./a0
140727554721148
140727554721152
94032535920704
