#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

__visible struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0x51d22b2b, __VMLINUX_SYMBOL_STR(module_layout) },
	{ 0x651c85c2, __VMLINUX_SYMBOL_STR(os_if_spin_unlock) },
	{ 0x71804050, __VMLINUX_SYMBOL_STR(os_if_queue_task) },
	{ 0x59b2ef9d, __VMLINUX_SYMBOL_STR(kmalloc_caches) },
	{ 0x499695e, __VMLINUX_SYMBOL_STR(vCanCleanup) },
	{ 0x51eafc8e, __VMLINUX_SYMBOL_STR(param_ops_int) },
	{ 0x70d738c9, __VMLINUX_SYMBOL_STR(os_if_spin_lock_irqsave) },
	{ 0x906447e8, __VMLINUX_SYMBOL_STR(queue_length) },
	{ 0xa974fd6f, __VMLINUX_SYMBOL_STR(dma_set_mask) },
	{ 0xef3feb6c, __VMLINUX_SYMBOL_STR(pci_disable_device) },
	{ 0x91831d70, __VMLINUX_SYMBOL_STR(seq_printf) },
	{ 0xf9658f03, __VMLINUX_SYMBOL_STR(os_if_read_lock_irqsave) },
	{ 0xee4beff9, __VMLINUX_SYMBOL_STR(pci_release_regions) },
	{ 0xa34324b7, __VMLINUX_SYMBOL_STR(os_if_read_unlock_irqrestore) },
	{ 0x593a99b, __VMLINUX_SYMBOL_STR(init_timer_key) },
	{ 0x91715312, __VMLINUX_SYMBOL_STR(sprintf) },
	{ 0x70c7c05f, __VMLINUX_SYMBOL_STR(os_if_spin_unlock_irqrestore) },
	{ 0x7d11c268, __VMLINUX_SYMBOL_STR(jiffies) },
	{ 0xbfc177bc, __VMLINUX_SYMBOL_STR(iowrite32_rep) },
	{ 0x170220be, __VMLINUX_SYMBOL_STR(queue_empty) },
	{ 0x70ba65d4, __VMLINUX_SYMBOL_STR(pci_set_master) },
	{ 0xd5f2172f, __VMLINUX_SYMBOL_STR(del_timer_sync) },
	{ 0xfb578fc5, __VMLINUX_SYMBOL_STR(memset) },
	{ 0xd5065308, __VMLINUX_SYMBOL_STR(queue_front) },
	{ 0x8accc30f, __VMLINUX_SYMBOL_STR(vCanDispatchEvent) },
	{ 0x4e926001, __VMLINUX_SYMBOL_STR(pci_iounmap) },
	{ 0x27e1a049, __VMLINUX_SYMBOL_STR(printk) },
	{ 0x449ad0a7, __VMLINUX_SYMBOL_STR(memcmp) },
	{ 0x4c9d28b0, __VMLINUX_SYMBOL_STR(phys_base) },
	{ 0x691087f9, __VMLINUX_SYMBOL_STR(queue_wakeup_on_space) },
	{ 0xdb8f4f1, __VMLINUX_SYMBOL_STR(os_if_delete_sema) },
	{ 0x6f580dd5, __VMLINUX_SYMBOL_STR(queue_pop) },
	{ 0x235ea4c1, __VMLINUX_SYMBOL_STR(calculateCRC32) },
	{ 0xbe2c0274, __VMLINUX_SYMBOL_STR(add_timer) },
	{ 0x2072ee9b, __VMLINUX_SYMBOL_STR(request_threaded_irq) },
	{ 0xc2781055, __VMLINUX_SYMBOL_STR(pci_clear_master) },
	{ 0xe27a3f6d, __VMLINUX_SYMBOL_STR(os_if_init_sema) },
	{ 0x1fe04677, __VMLINUX_SYMBOL_STR(os_if_rwlock_remove) },
	{ 0x1230ccf6, __VMLINUX_SYMBOL_STR(vCanInitData) },
	{ 0x451c7bd4, __VMLINUX_SYMBOL_STR(os_if_kernel_malloc) },
	{ 0x8efbe361, __VMLINUX_SYMBOL_STR(os_if_write_lock_irqsave) },
	{ 0xdb7305a1, __VMLINUX_SYMBOL_STR(__stack_chk_fail) },
	{ 0x3bd1b1f6, __VMLINUX_SYMBOL_STR(msecs_to_jiffies) },
	{ 0xb72ec502, __VMLINUX_SYMBOL_STR(os_if_down_sema) },
	{ 0xbdfb6dbb, __VMLINUX_SYMBOL_STR(__fentry__) },
	{ 0x966f779f, __VMLINUX_SYMBOL_STR(pci_unregister_driver) },
	{ 0x71b953bf, __VMLINUX_SYMBOL_STR(kmem_cache_alloc_trace) },
	{ 0x9338dd27, __VMLINUX_SYMBOL_STR(os_if_kernel_free) },
	{ 0x2b65971a, __VMLINUX_SYMBOL_STR(queue_irq_lock) },
	{ 0xfa49442f, __VMLINUX_SYMBOL_STR(os_if_set_task_uninterruptible) },
	{ 0x37a0cba, __VMLINUX_SYMBOL_STR(kfree) },
	{ 0x69acdf38, __VMLINUX_SYMBOL_STR(memcpy) },
	{ 0x4570f64e, __VMLINUX_SYMBOL_STR(pci_request_regions) },
	{ 0xc5a48594, __VMLINUX_SYMBOL_STR(os_if_rwlock_init) },
	{ 0x91e9b047, __VMLINUX_SYMBOL_STR(vCanInit) },
	{ 0xc226b024, __VMLINUX_SYMBOL_STR(os_if_up_sema) },
	{ 0x6783e368, __VMLINUX_SYMBOL_STR(__pci_register_driver) },
	{ 0xb9ac4148, __VMLINUX_SYMBOL_STR(os_if_init_waitqueue_head) },
	{ 0x9a56bfc2, __VMLINUX_SYMBOL_STR(queue_reinit) },
	{ 0x6e94147d, __VMLINUX_SYMBOL_STR(queue_release) },
	{ 0x371e4ec3, __VMLINUX_SYMBOL_STR(os_if_init_task) },
	{ 0x4a2261d9, __VMLINUX_SYMBOL_STR(os_if_wait_for_event_timeout_simple) },
	{ 0xe058d58b, __VMLINUX_SYMBOL_STR(pci_iomap) },
	{ 0xdb5bb4d0, __VMLINUX_SYMBOL_STR(os_if_spin_lock) },
	{ 0x436c2179, __VMLINUX_SYMBOL_STR(iowrite32) },
	{ 0x1cce8c94, __VMLINUX_SYMBOL_STR(pci_enable_device) },
	{ 0xd742d12f, __VMLINUX_SYMBOL_STR(os_if_spin_lock_remove) },
	{ 0x36efa166, __VMLINUX_SYMBOL_STR(os_if_write_unlock_irqrestore) },
	{ 0x10ce0023, __VMLINUX_SYMBOL_STR(dma_ops) },
	{ 0xbac4b01c, __VMLINUX_SYMBOL_STR(os_if_wake_up_interruptible) },
	{ 0xe484e35f, __VMLINUX_SYMBOL_STR(ioread32) },
	{ 0xf20dabd8, __VMLINUX_SYMBOL_STR(free_irq) },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=kvcommon";


MODULE_INFO(srcversion, "E3E627BE7FA9DB32E6C3CF8");
