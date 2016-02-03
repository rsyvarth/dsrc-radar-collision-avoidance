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
	{ 0x52e6e992, __VMLINUX_SYMBOL_STR(queue_init) },
	{ 0x499695e, __VMLINUX_SYMBOL_STR(vCanCleanup) },
	{ 0x51eafc8e, __VMLINUX_SYMBOL_STR(param_ops_int) },
	{ 0x70d738c9, __VMLINUX_SYMBOL_STR(os_if_spin_lock_irqsave) },
	{ 0x906447e8, __VMLINUX_SYMBOL_STR(queue_length) },
	{ 0x3fb8a666, __VMLINUX_SYMBOL_STR(seq_puts) },
	{ 0x91831d70, __VMLINUX_SYMBOL_STR(seq_printf) },
	{ 0xb737886e, __VMLINUX_SYMBOL_STR(os_if_init_waitqueue_entry) },
	{ 0x416477ab, __VMLINUX_SYMBOL_STR(usb_kill_urb) },
	{ 0x5740a430, __VMLINUX_SYMBOL_STR(os_if_wait_for_event_timeout) },
	{ 0x593a99b, __VMLINUX_SYMBOL_STR(init_timer_key) },
	{ 0x70c7c05f, __VMLINUX_SYMBOL_STR(os_if_spin_unlock_irqrestore) },
	{ 0x7d11c268, __VMLINUX_SYMBOL_STR(jiffies) },
	{ 0x2f868165, __VMLINUX_SYMBOL_STR(os_if_destroy_task) },
	{ 0x170220be, __VMLINUX_SYMBOL_STR(queue_empty) },
	{ 0xbe1e2602, __VMLINUX_SYMBOL_STR(queue_push) },
	{ 0xd5f2172f, __VMLINUX_SYMBOL_STR(del_timer_sync) },
	{ 0xfb578fc5, __VMLINUX_SYMBOL_STR(memset) },
	{ 0xd5065308, __VMLINUX_SYMBOL_STR(queue_front) },
	{ 0x28d32f9a, __VMLINUX_SYMBOL_STR(queue_add_wait_for_space) },
	{ 0x8accc30f, __VMLINUX_SYMBOL_STR(vCanDispatchEvent) },
	{ 0xf32a6ff4, __VMLINUX_SYMBOL_STR(usb_deregister) },
	{ 0x27e1a049, __VMLINUX_SYMBOL_STR(printk) },
	{ 0x691087f9, __VMLINUX_SYMBOL_STR(queue_wakeup_on_space) },
	{ 0xdb8f4f1, __VMLINUX_SYMBOL_STR(os_if_delete_sema) },
	{ 0x6f580dd5, __VMLINUX_SYMBOL_STR(queue_pop) },
	{ 0x12aee849, __VMLINUX_SYMBOL_STR(os_if_kernel_thread) },
	{ 0xbe2c0274, __VMLINUX_SYMBOL_STR(add_timer) },
	{ 0x56288c39, __VMLINUX_SYMBOL_STR(usb_free_coherent) },
	{ 0xe27a3f6d, __VMLINUX_SYMBOL_STR(os_if_init_sema) },
	{ 0x1230ccf6, __VMLINUX_SYMBOL_STR(vCanInitData) },
	{ 0x92558135, __VMLINUX_SYMBOL_STR(usb_submit_urb) },
	{ 0x451c7bd4, __VMLINUX_SYMBOL_STR(os_if_kernel_malloc) },
	{ 0xad5568b8, __VMLINUX_SYMBOL_STR(os_if_set_task_interruptible) },
	{ 0xdb7305a1, __VMLINUX_SYMBOL_STR(__stack_chk_fail) },
	{ 0x3bd1b1f6, __VMLINUX_SYMBOL_STR(msecs_to_jiffies) },
	{ 0x8b31d4ef, __VMLINUX_SYMBOL_STR(usb_bulk_msg) },
	{ 0xbdf05c0f, __VMLINUX_SYMBOL_STR(os_if_queue_task_not_default_queue) },
	{ 0xb72ec502, __VMLINUX_SYMBOL_STR(os_if_down_sema) },
	{ 0xdbdf4d78, __VMLINUX_SYMBOL_STR(os_if_exit_thread) },
	{ 0xbdfb6dbb, __VMLINUX_SYMBOL_STR(__fentry__) },
	{ 0x9338dd27, __VMLINUX_SYMBOL_STR(os_if_kernel_free) },
	{ 0xaa836602, __VMLINUX_SYMBOL_STR(os_if_signal_pending) },
	{ 0x801678, __VMLINUX_SYMBOL_STR(flush_scheduled_work) },
	{ 0x91e9b047, __VMLINUX_SYMBOL_STR(vCanInit) },
	{ 0xc226b024, __VMLINUX_SYMBOL_STR(os_if_up_sema) },
	{ 0xc13d886a, __VMLINUX_SYMBOL_STR(os_if_set_task_running) },
	{ 0x90e986df, __VMLINUX_SYMBOL_STR(usb_register_driver) },
	{ 0x9d609c8c, __VMLINUX_SYMBOL_STR(queue_remove_wait_for_space) },
	{ 0x34538454, __VMLINUX_SYMBOL_STR(queue_back) },
	{ 0x9a56bfc2, __VMLINUX_SYMBOL_STR(queue_reinit) },
	{ 0x6e94147d, __VMLINUX_SYMBOL_STR(queue_release) },
	{ 0x371e4ec3, __VMLINUX_SYMBOL_STR(os_if_init_task) },
	{ 0x4a2261d9, __VMLINUX_SYMBOL_STR(os_if_wait_for_event_timeout_simple) },
	{ 0x361b484a, __VMLINUX_SYMBOL_STR(usb_alloc_coherent) },
	{ 0xdb5bb4d0, __VMLINUX_SYMBOL_STR(os_if_spin_lock) },
	{ 0x149d5959, __VMLINUX_SYMBOL_STR(os_if_declare_task) },
	{ 0xd742d12f, __VMLINUX_SYMBOL_STR(os_if_spin_lock_remove) },
	{ 0xccf52a8f, __VMLINUX_SYMBOL_STR(usb_free_urb) },
	{ 0xbac4b01c, __VMLINUX_SYMBOL_STR(os_if_wake_up_interruptible) },
	{ 0x86ac3786, __VMLINUX_SYMBOL_STR(try_module_get) },
	{ 0x4ec18d9e, __VMLINUX_SYMBOL_STR(usb_alloc_urb) },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=kvcommon";

MODULE_ALIAS("usb:v0BFDp0004d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0BFDp0002d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0BFDp0005d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0BFDp0003d*dc*dsc*dp*ic*isc*ip*in*");

MODULE_INFO(srcversion, "C8ED4600B4ED29E34000912");
