//===
static int __ref kernel_init(void *unused)
{
	int ret;

	/*
	 * Wait until kthreadd is all set-up.
	 */
	wait_for_completion(&kthreadd_done);

	kernel_init_freeable();
	/* need to finish all async __init code before freeing the memory */
	async_synchronize_full();

	system_state = SYSTEM_FREEING_INITMEM;
	kprobe_free_init_mem();
	ftrace_free_init_mem();
	kgdb_free_init_mem();
	exit_boot_config();
	free_initmem();
	mark_readonly();

	/*
	 * Kernel mappings are now finalized - update the userspace page-table
	 * to finalize PTI.
	 */
	pti_finalize();

	system_state = SYSTEM_RUNNING;
	numa_default_policy();

	rcu_end_inkernel_boot();

	do_sysctl_args();

	if (ramdisk_execute_command) {
		ret = run_init_process(ramdisk_execute_command);
		if (!ret)
			return 0;
		pr_err("Failed to execute %s (error %d)\n",
		       ramdisk_execute_command, ret);
	}

	/*
	 * We try each of these until one succeeds.
	 *
	 * The Bourne shell can be used instead of init if we are
	 * trying to recover a really broken machine.
	 */
	if (execute_command) {
		ret = run_init_process(execute_command);
		if (!ret)
			return 0;
		panic("Requested init %s failed (error %d).",
		      execute_command, ret);
	}

	if (CONFIG_DEFAULT_INIT[0] != '\0') {
		ret = run_init_process(CONFIG_DEFAULT_INIT);
		if (ret)
			pr_err("Default init %s failed (error %d)\n",
			       CONFIG_DEFAULT_INIT, ret);
		else
			return 0;
	}

	if (!try_to_run_init_process("/sbin/init") ||
	    !try_to_run_init_process("/etc/init") ||
	    !try_to_run_init_process("/bin/init") ||
	    !try_to_run_init_process("/bin/sh"))
		return 0;

	panic("No working init found.  Try passing init= option to kernel. "
	      "See Linux Documentation/admin-guide/init.rst for guidance.");
}

//==
struct alpha_machine_vector
{
	/* This "belongs" down below with the rest of the runtime
	   variables, but it is convenient for entry.S if these
	   two slots are at the beginning of the struct.  */
	unsigned long hae_cache;
	unsigned long *hae_register;

	int nr_irqs;
	int rtc_port;
	int rtc_boot_cpu_only;
	unsigned int max_asn;
	unsigned long max_isa_dma_address;
	unsigned long irq_probe_mask;
	unsigned long iack_sc;
	unsigned long min_io_address;
	unsigned long min_mem_address;
	unsigned long pci_dac_offset;

	void (*mv_pci_tbi)(struct pci_controller *hose,
			   dma_addr_t start, dma_addr_t end);

	unsigned int (*mv_ioread8)(const void __iomem *);
	unsigned int (*mv_ioread16)(const void __iomem *);
	unsigned int (*mv_ioread32)(const void __iomem *);

	void (*mv_iowrite8)(u8, void __iomem *);
	void (*mv_iowrite16)(u16, void __iomem *);
	void (*mv_iowrite32)(u32, void __iomem *);

	u8 (*mv_readb)(const volatile void __iomem *);
	u16 (*mv_readw)(const volatile void __iomem *);
	u32 (*mv_readl)(const volatile void __iomem *);
	u64 (*mv_readq)(const volatile void __iomem *);

	void (*mv_writeb)(u8, volatile void __iomem *);
	void (*mv_writew)(u16, volatile void __iomem *);
	void (*mv_writel)(u32, volatile void __iomem *);
	void (*mv_writeq)(u64, volatile void __iomem *);

	void __iomem *(*mv_ioportmap)(unsigned long);
	void __iomem *(*mv_ioremap)(unsigned long, unsigned long);
	void (*mv_iounmap)(volatile void __iomem *);
	int (*mv_is_ioaddr)(unsigned long);
	int (*mv_is_mmio)(const volatile void __iomem *);

	void (*mv_switch_mm)(struct mm_struct *, struct mm_struct *,
			     struct task_struct *);
	void (*mv_activate_mm)(struct mm_struct *, struct mm_struct *);

	void (*mv_flush_tlb_current)(struct mm_struct *);
	void (*mv_flush_tlb_current_page)(struct mm_struct * mm,
					  struct vm_area_struct *vma,
					  unsigned long addr);

	void (*update_irq_hw)(unsigned long, unsigned long, int);
	void (*ack_irq)(unsigned long);
	void (*device_interrupt)(unsigned long vector);
	void (*machine_check)(unsigned long vector, unsigned long la);

	void (*smp_callin)(void);
	void (*init_arch)(void);
	void (*init_irq)(void);
	void (*init_rtc)(void);
	void (*init_pci)(void);
	void (*kill_arch)(int);

	u8 (*pci_swizzle)(struct pci_dev *, u8 *);
	int (*pci_map_irq)(const struct pci_dev *, u8, u8);
	struct pci_ops *pci_ops;

	struct _alpha_agp_info *(*agp_info)(void);

	const char *vector_name;

	/* System specific parameters.  */
	union {
	    struct {
		unsigned long gru_int_req_bits;
	    } cia;

	    struct {
		unsigned long gamma_bias;
	    } t2;

	    struct {
		unsigned int route_tab;
	    } sio;
	} sys;
};

//====
time_init(void)
{
	unsigned int cc1, cc2;
	unsigned long cycle_freq, tolerance;
	long diff;

	if (alpha_using_qemu) {
		clocksource_register_hz(&qemu_cs, NSEC_PER_SEC);
		init_qemu_clockevent();
		init_rtc_irq(qemu_timer_interrupt);
		return;
	}

	/* Calibrate CPU clock -- attempt #1.  */
	if (!est_cycle_freq)
		est_cycle_freq = validate_cc_value(calibrate_cc_with_pit());

	cc1 = rpcc();

	/* Calibrate CPU clock -- attempt #2.  */
	if (!est_cycle_freq) {
		cc1 = rpcc_after_update_in_progress();
		cc2 = rpcc_after_update_in_progress();
		est_cycle_freq = validate_cc_value(cc2 - cc1);
		cc1 = cc2;
	}

	cycle_freq = hwrpb->cycle_freq;
	if (est_cycle_freq) {
		/* If the given value is within 250 PPM of what we calculated,
		   accept it.  Otherwise, use what we found.  */
		tolerance = cycle_freq / 4000;
		diff = cycle_freq - est_cycle_freq;
		if (diff < 0)
			diff = -diff;
		if ((unsigned long)diff > tolerance) {
			cycle_freq = est_cycle_freq;
			printk("HWRPB cycle frequency bogus.  "
			       "Estimated %lu Hz\n", cycle_freq);
		} else {
			est_cycle_freq = 0;
		}
	} else if (! validate_cc_value (cycle_freq)) {
		printk("HWRPB cycle frequency bogus, "
		       "and unable to estimate a proper value!\n");
	}

	/* See above for restrictions on using clocksource_rpcc.  */
#ifndef CONFIG_ALPHA_WTINT
	if (hwrpb->nr_processors == 1)
		clocksource_register_hz(&clocksource_rpcc, cycle_freq);
#endif

	/* Startup the timer source. */
	alpha_mv.init_rtc();
	init_rtc_clockevent();
}

