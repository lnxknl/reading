activate:
	/* save kernel state */
	mrs ip, psr
	push {r4, r5, r6, r7, r8, r9, r10, r11, ip, lr}

	/* load user state */
	ldmia r0!, {r4, r5, r6, r7, r8, r9, r10, r11, lr}

	msr psp, r0

	/* jump to user task */
	bx lr

void task_init(void)
{
	unsigned int null_stacks[32];
	init_activate_env(&null_stacks[32]);
}
