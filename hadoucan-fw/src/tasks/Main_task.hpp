#pragma once

#include "freertos_cpp_util/Task_static.hpp"

#include "CAN_USB_app.hpp"

class Main_task : public Task_static<2048>
{
public:
	void work() override;

	bool init_usb();

	bool mount_fs();

	bool load_config();

	static bool handle_usb_set_config_thunk(void* ctx, const uint16_t config);

protected:
	
	void test_lfs_move_handle();

	void halt_on_error();

	bool handle_usb_set_config(const uint8_t config);

	void get_unique_id(std::array<uint32_t, 3>* id);
	void get_unique_id_str(std::array<char, 25>* id_str);

	std::array<char, 25> usb_id_str;//this is read by the usb core, and sent as a descriptor payload
};
