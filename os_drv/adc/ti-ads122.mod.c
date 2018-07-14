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
	{ 0xd29ace7d, __VMLINUX_SYMBOL_STR(module_layout) },
	{ 0x6f3728e5, __VMLINUX_SYMBOL_STR(iio_read_const_attr) },
	{ 0x6fc29b59, __VMLINUX_SYMBOL_STR(i2c_unregister_device) },
	{ 0x5d1d71ea, __VMLINUX_SYMBOL_STR(i2c_del_driver) },
	{ 0x31f5e72b, __VMLINUX_SYMBOL_STR(i2c_register_driver) },
	{ 0x6fb68dd1, __VMLINUX_SYMBOL_STR(i2c_put_adapter) },
	{ 0x2eb03c8b, __VMLINUX_SYMBOL_STR(i2c_new_device) },
	{ 0x75555e97, __VMLINUX_SYMBOL_STR(i2c_get_adapter) },
	{ 0x16305289, __VMLINUX_SYMBOL_STR(warn_slowpath_null) },
	{ 0xc29548a6, __VMLINUX_SYMBOL_STR(__pm_runtime_resume) },
	{ 0x172f021e, __VMLINUX_SYMBOL_STR(__pm_runtime_suspend) },
	{ 0x7d11c268, __VMLINUX_SYMBOL_STR(jiffies) },
	{ 0x1ace1deb, __VMLINUX_SYMBOL_STR(regmap_get_device) },
	{ 0xbc2570d7, __VMLINUX_SYMBOL_STR(regmap_write) },
	{ 0x2bb81a76, __VMLINUX_SYMBOL_STR(regmap_read) },
	{ 0x9a18d7ff, __VMLINUX_SYMBOL_STR(dev_err) },
	{ 0x192441c6, __VMLINUX_SYMBOL_STR(hrtimer_init) },
	{ 0xb13fee78, __VMLINUX_SYMBOL_STR(iio_device_register) },
	{ 0xa2324d5b, __VMLINUX_SYMBOL_STR(iio_buffer_get) },
	{ 0xe7dce76d, __VMLINUX_SYMBOL_STR(iio_kfifo_allocate) },
	{ 0x12a38747, __VMLINUX_SYMBOL_STR(usleep_range) },
	{ 0x4bdd5b02, __VMLINUX_SYMBOL_STR(i2c_transfer) },
	{ 0xff227eee, __VMLINUX_SYMBOL_STR(__devm_regmap_init_i2c) },
	{ 0x9d669763, __VMLINUX_SYMBOL_STR(memcpy) },
	{ 0xf090eadf, __VMLINUX_SYMBOL_STR(__mutex_init) },
	{ 0xfa48f66d, __VMLINUX_SYMBOL_STR(devm_iio_device_alloc) },
	{ 0x6d662533, __VMLINUX_SYMBOL_STR(_find_first_bit_le) },
	{ 0x9955a327, __VMLINUX_SYMBOL_STR(hrtimer_start_range_ns) },
	{ 0x11025677, __VMLINUX_SYMBOL_STR(hrtimer_cancel) },
	{ 0x4205ad24, __VMLINUX_SYMBOL_STR(cancel_work_sync) },
	{ 0x5b19634d, __VMLINUX_SYMBOL_STR(div_s64_rem) },
	{ 0x2f1f8f20, __VMLINUX_SYMBOL_STR(mutex_unlock) },
	{ 0x9138e27f, __VMLINUX_SYMBOL_STR(mutex_lock) },
	{ 0xe707d823, __VMLINUX_SYMBOL_STR(__aeabi_uidiv) },
	{ 0x27e1a049, __VMLINUX_SYMBOL_STR(printk) },
	{ 0x3914c34e, __VMLINUX_SYMBOL_STR(regmap_bulk_read) },
	{ 0xc5980f27, __VMLINUX_SYMBOL_STR(iio_get_time_ns) },
	{ 0xf185456e, __VMLINUX_SYMBOL_STR(iio_device_unregister) },
	{ 0xf6935a11, __VMLINUX_SYMBOL_STR(regmap_update_bits_base) },
	{ 0xfe009c06, __VMLINUX_SYMBOL_STR(hrtimer_forward) },
	{ 0xb2d48a2e, __VMLINUX_SYMBOL_STR(queue_work_on) },
	{ 0x2d3385d3, __VMLINUX_SYMBOL_STR(system_wq) },
	{ 0x2e5810c6, __VMLINUX_SYMBOL_STR(__aeabi_unwind_cpp_pr1) },
	{ 0xb1ad28e0, __VMLINUX_SYMBOL_STR(__gnu_mcount_nc) },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=industrialio,kfifo_buf";

MODULE_ALIAS("i2c:ads122-0");
MODULE_ALIAS("i2c:ads122-1");

MODULE_INFO(srcversion, "DA095F305EE144318C4C4F4");
