add:
.LFB0:
        .cfi_startproc
        pushq   %rbp
        .cfi_def_cfa_offset 16
        .cfi_offset 6, -16
        movq    %rsp, %rbp
        .cfi_def_cfa_register 6
        movl    %edi, -4(%rbp)
        movl    %esi, -8(%rbp)
        movl    -4(%rbp), %edx
        movl    -8(%rbp), %eax
        addl    %edx, %eax
        popq    %rbp
        .cfi_def_cfa 7, 8
        ret
        .cfi_endproc
.LFE0:
        .size   add, .-add
        .globl  main
        .type   main, @function
main:
.LFB1:
        .cfi_startproc
        pushq   %rbp
        .cfi_def_cfa_offset 16
        .cfi_offset 6, -16
        movq    %rsp, %rbp
        .cfi_def_cfa_register 6
        subq    $16, %rsp
        movl    $3, -12(%rbp)
        movl    $3, -8(%rbp)
        movl    -8(%rbp), %edx
        movl    -12(%rbp), %eax
        movl    %edx, %esi
        movl    %eax, %edi
        call    add
        movl    %eax, -4(%rbp)
        movl    $0, %eax
        leave
        .cfi_def_cfa 7, 8
        ret
        .cfi_endproc


(gdb) si
(gdb) p $rsp
$7 = (void *) 0x7fffffffcff0
(gdb) si
add (a=0, b=-1075053569) at ./t0.c:4
(gdb) p $rsp
$8 = (void *) 0x7fffffffcfe8

## rsp auto increase


Breakpoint 3, add (a=0x7fffffffcfe8, b=3) at ./t0.c:4
(gdb) si
(gdb) si
(gdb) si
(gdb) si
(gdb) xac $rax 10
0x3: 0xCannot access memory at address 0x3
(gdb) si
(gdb) xac $rax 10
0x7fffffffcfe8: 0x03 0x00 0x00 0x00 0x00 0x00 0x00 0x00 '^C'
0x7fffffffcff0: 0xe8 0xcf 'M-O'
(gdb) p $rax
$24 = 140737488343016
(gdb) winheight cmd + 10
(gdb) p /x $rax
$25 = 0x7fffffffcfe8
(gdb) p $rdx
$26 = 6
(gdb) si
(gdb) p /x $rax
$27 = 0x7fffffffcfe8
(gdb) xac $rax 10
0x7fffffffcfe8: 0x06 0x00 0x00 0x00 0x00 0x00 0x00 0x00 '^F'
0x7fffffffcff0: 0xe8 0xcf 'M-O'
