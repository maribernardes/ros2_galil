#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "galil_driver/hardware_interface.hpp"

namespace galil_driver {

	GalilSystemHardwareInterface::~GalilSystemHardwareInterface(){
		on_deactivate(rclcpp_lifecycle::State());
	}
  
	hardware_interface::CallbackReturn
	//GalilSystemHardwareInterface::on_init(const hardware_interface::HardwareInfo& info){ //Mariana: Jazzy fix
	GalilSystemHardwareInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams & params){
		cmd_mode_ = 0;

		/*if( hardware_interface::SystemInterface::on_init(info) !=
		hardware_interface::CallbackReturn::SUCCESS ){
			return hardware_interface::CallbackReturn::ERROR;
		}
		*/ 
		//Mariana: Jazzy fix:
		// Call base class version with the new params type
		if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS){
			return hardware_interface::CallbackReturn::ERROR;
		}

		//info_ = info; //Mariana: Jazzy fix
		// The HardwareInfo now comes from params.hardware_info - Mariana: Jazzy fix
		info_ = params.hardware_info;

		hw_states_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
		hw_states_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
		hw_states_effort_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
		hw_commands_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
		hw_commands_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

		for (const hardware_interface::ComponentInfo & joint : info_.joints){
			for (std::size_t i=0; i<joint.command_interfaces.size(); i++ ){
				if (joint.command_interfaces[i].name == hardware_interface::HW_IF_POSITION)
					RCLCPP_INFO(rclcpp::get_logger("GalilSystemHardwareInterface"), " %s has position command interface.", joint.name.c_str() );
				if (joint.command_interfaces[i].name == hardware_interface::HW_IF_VELOCITY)
					RCLCPP_INFO(rclcpp::get_logger("GalilSystemHardwareInterface"), " %s has velocity command interface.", joint.name.c_str() );
			}
			for (std::size_t i=0; i<joint.state_interfaces.size(); i++ ){
				if (joint.state_interfaces[i].name == hardware_interface::HW_IF_POSITION)
					RCLCPP_INFO(rclcpp::get_logger("GalilSystemHardwareInterface"), " %s has position state interface.", joint.name.c_str() );
				if (joint.state_interfaces[i].name == hardware_interface::HW_IF_VELOCITY)
					RCLCPP_INFO(rclcpp::get_logger("GalilSystemHardwareInterface"), " %s has velocity state interface.", joint.name.c_str() );
				if (joint.state_interfaces[i].name == hardware_interface::HW_IF_EFFORT)
					RCLCPP_INFO(rclcpp::get_logger("GalilSystemHardwareInterface"), " has effort state interface.");
			}
		}
		return hardware_interface::CallbackReturn::SUCCESS;
	}

	hardware_interface::CallbackReturn
	GalilSystemHardwareInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/){
		RCLCPP_INFO(rclcpp::get_logger("GalilSystemHardwareInterface"), "Configuring ...please wait...");

		std::string address("192.168.0.99");
		if (GOpen( address.c_str(), &connection ) == G_NO_ERROR ){
			RCLCPP_INFO(rclcpp::get_logger("GalilSystemHardwareInterface"), "Connection to Galil successful.");
		}
		else {
			RCLCPP_ERROR(rclcpp::get_logger("GalilSystemHardwareInterface"), "Failed to open connection with Galil.");
			return hardware_interface::CallbackReturn::ERROR;
		}

		// The NEW BWH order: X = horizontal (CH A), Y = insertion (CH B), Z = vertical (CH C)
		// This should be in on_init and search for joint.name instead
		// Likewise, gears should use URDF transmission
		gears_m_2_cnt.push_back(1000.0*715.0);        // horizontal - CH A
		gears_m_2_cnt.push_back(1000.0*-2000.0/1.27); // insertion - CH B
		gears_m_2_cnt.push_back(1000.0*1430.0);       // vertical - CH C

		RCLCPP_INFO(rclcpp::get_logger("GalilSystemHardwareInterface"), "Successfully configured!");
		return hardware_interface::CallbackReturn::SUCCESS;
	}
  
	std::vector<hardware_interface::StateInterface> GalilSystemHardwareInterface::export_state_interfaces(){
		std::vector<hardware_interface::StateInterface> state_interfaces;
		for (std::size_t i=0; i<info_.joints.size(); i++){
			state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name,
											hardware_interface::HW_IF_POSITION,
											&hw_states_position_[i]) );
			state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name,
											hardware_interface::HW_IF_VELOCITY,
											&hw_states_velocity_[i]) );
			state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name,
											hardware_interface::HW_IF_EFFORT,
											&hw_states_effort_[i]) );
		}
		return state_interfaces;
	}
  
	std::vector<hardware_interface::CommandInterface> GalilSystemHardwareInterface::export_command_interfaces(){
		std::vector<hardware_interface::CommandInterface> command_interfaces;
		for (std::size_t i=0; i<info_.joints.size(); i++)
		{
			command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name,
											hardware_interface::HW_IF_POSITION,
											&hw_commands_position_[i]) );
			command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name,
											hardware_interface::HW_IF_VELOCITY,
											&hw_commands_velocity_[i]) );
		}
		return command_interfaces;
	}
  
	hardware_interface::CallbackReturn
	GalilSystemHardwareInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/){
		return hardware_interface::CallbackReturn::SUCCESS;
	}
  
	hardware_interface::CallbackReturn
	GalilSystemHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/){
		return hardware_interface::CallbackReturn::SUCCESS;
	}
  
	hardware_interface::return_type
	GalilSystemHardwareInterface::prepare_command_mode_switch(const std::vector<std::string>& start_interfaces,
															const std::vector<std::string>& stop_interfaces){
		RCLCPP_INFO(rclcpp::get_logger("GalilSystemHardwareInterface"),"Prepare command mode switch.");

		hardware_interface::return_type ret_val = hardware_interface::return_type::OK;

		{ // brute force. not sure about the other loops
			GSize BUFFER_LENGTH=1024;
			GSize bytes_returned;
			char command[1024]="";
			char buffer[1024];
			sprintf( command, "ST");
			RCLCPP_INFO( rclcpp::get_logger("GalilSystemHardwareInterface"),"Stopping Galil with ST.");
			int error = GCommand(connection, command, buffer, BUFFER_LENGTH, &bytes_returned);
			if( error != G_NO_ERROR ){
				RCLCPP_ERROR(rclcpp::get_logger("GalilSystemHardwareInterface"),
							"Failed to send command: %s, error code %d", command, error);
			}
		}

		// Process the interfaces to stop.
		for (const auto& key : stop_interfaces){
			for (auto i = 0u; i < info_.joints.size(); i++){
				if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION){
					RCLCPP_INFO_STREAM( rclcpp::get_logger("GalilSystemHardwareInterface"),
								"Preparing to stop interface " <<
								info_.joints[i].name <<
								"/" <<  hardware_interface::HW_IF_POSITION );
					
					hw_commands_velocity_[i] = 0.0;

					GSize BUFFER_LENGTH=1024;
					GSize bytes_returned;
					char channels[] = "ABCD";
					char command[1024]="";
					char buffer[1024];
					sprintf( command, "ST %c", channels[i]);
					int error = GCommand(connection, command, buffer, BUFFER_LENGTH, &bytes_returned );
					if (error != G_NO_ERROR){
						RCLCPP_ERROR( rclcpp::get_logger("GalilSystemHardwareInterface"),
									"Failed to send command: %s, error code %d", command, error);
						return hardware_interface::return_type::ERROR;
					}
				}

				// for velocity we must send stop command
				if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY){
					RCLCPP_INFO_STREAM( rclcpp::get_logger("GalilSystemHardwareInterface"),
								"Preparin to stop interface " <<
								info_.joints[i].name <<
								"/" <<  hardware_interface::HW_IF_VELOCITY );

					hw_commands_velocity_[i] = 0.0;

					GSize BUFFER_LENGTH=1024;
					GSize bytes_returned;
					char channels[] = "ABCD";
					char command[1024]="";
					char buffer[1024];
					sprintf( command, "ST %c", channels[i]);
					int error = GCommand(connection, command, buffer, BUFFER_LENGTH, &bytes_returned);
					if (error != G_NO_ERROR){
						RCLCPP_ERROR( rclcpp::get_logger("GalilSystemHardwareInterface"),
									"Failed to send command: %s, error code %d", command, error);
						return hardware_interface::return_type::ERROR;
					}
				}
			}
		}

		for (const auto& key : start_interfaces){
			for (auto i = 0u; i < info_.joints.size(); i++){
				if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION){
					RCLCPP_INFO_STREAM( rclcpp::get_logger("GalilSystemHardwareInterface"),
								"Preparing to start interface " <<
								info_.joints[i].name <<
								"/" <<  hardware_interface::HW_IF_POSITION );
					cmd_mode_ = 1;
					hw_commands_velocity_[i] = 0.0;
				}
				if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY){
					RCLCPP_INFO_STREAM( rclcpp::get_logger("GalilSystemHardwareInterface"),
								"Preparing to start interface " <<
								info_.joints[i].name <<
								"/" <<  hardware_interface::HW_IF_VELOCITY );
					cmd_mode_ = 2;
					hw_commands_velocity_[i] = 0.0;
				}
			}
		}
		return ret_val;
	}
  
	hardware_interface::return_type
	GalilSystemHardwareInterface::perform_command_mode_switch(const std::vector<std::string>& start_interfaces,
															const std::vector<std::string>& stop_interfaces){

		std::cout << "GalilSystemHardwareInterface::perform_command_mode_switch" << std::endl;
		hardware_interface::return_type ret_val = hardware_interface::return_type::OK;

		for (const auto& key : stop_interfaces){
			for (auto i = 0u; i < info_.joints.size(); i++){
				if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION){
					RCLCPP_INFO_STREAM( rclcpp::get_logger("GalilSystemHardwareInterface"),
								"Stop interface " <<
								info_.joints[i].name <<
								"/" <<  hardware_interface::HW_IF_POSITION );
					hw_commands_velocity_[i] = 0.0;
				}
				if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY){
					RCLCPP_INFO_STREAM( rclcpp::get_logger("GalilSystemHardwareInterface"),
								"Stop interface " <<
								info_.joints[i].name <<
								"/" <<  hardware_interface::HW_IF_VELOCITY );
					hw_commands_velocity_[i] = 0.0;
				}
			}
		}

		for (const auto& key : start_interfaces){
			for (auto i = 0u; i < info_.joints.size(); i++) {
				if(key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION){
					RCLCPP_INFO_STREAM( rclcpp::get_logger("GalilSystemHardwareInterface"),
								"Start interface " <<
								info_.joints[i].name <<
								"/" <<  hardware_interface::HW_IF_POSITION );
					// This is to prevent sending old position when switching
					hw_commands_position_[i] = hw_states_position_[i];
					hw_commands_velocity_[i] = 0.0;

					GSize BUFFER_LENGTH=1024;
					GSize bytes_returned;
					char command[1024]="";
					char buffer[1024];
					sprintf( command, "PT 1,1,1,1" ); // no need to send that command multiple times
					int error = GCommand(connection, command, buffer, BUFFER_LENGTH, &bytes_returned );
					if (error != G_NO_ERROR ){
						RCLCPP_ERROR( rclcpp::get_logger("GalilSystemHardwareInterface"),
									"Failed to send command: %s, error code %d", command, error);
						return hardware_interface::return_type::ERROR;
					}
				}
				if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY){
					RCLCPP_INFO_STREAM( rclcpp::get_logger("GalilSystemHardwareInterface"),
								"Start interface " <<
								info_.joints[i].name <<
								"/" <<  hardware_interface::HW_IF_VELOCITY );
					hw_commands_velocity_[i] = 0.0;

					GSize BUFFER_LENGTH=1024;
					GSize bytes_returned;
					char command[1024]="";
					char buffer[1024];
					sprintf(command, "PT 1,1,1,1"); // no need to send that command multiple times
					int error = GCommand(connection, command, buffer, BUFFER_LENGTH, &bytes_returned );
					if (error != G_NO_ERROR){
						RCLCPP_ERROR( rclcpp::get_logger("GalilSystemHardwareInterface"),
									"Failed to send command: %s, error code %d", command, error);
						return hardware_interface::return_type::ERROR;
					}
				}
      		}
    	}
    	return ret_val;
  	}

  
  	hardware_interface::return_type GalilSystemHardwareInterface::read(const rclcpp::Time& /*time*/,
								      								const rclcpp::Duration& /*period*/){
		GSize BUFFER_LENGTH=1024;
		GSize bytes_returned;
		char buffer[1024];
		if (GCommand(connection, "TP", buffer, BUFFER_LENGTH, &bytes_returned ) == G_NO_ERROR){
			char * pch;
			pch = strtok(buffer,",");
			for (std::size_t i=0; i<info_.joints.size(); i++ ){
				if(pch != NULL){
					hw_states_position_[i] = atof( pch )/gears_m_2_cnt[i];
					pch = strtok (NULL, ",");
				}
			}
		}
		else {
			RCLCPP_ERROR(rclcpp::get_logger("GalilSystemHardwareInterface"), "Failed to send TP command to Galil.");
			return hardware_interface::return_type::ERROR;
		}
		if(GCommand(connection, "TV", buffer, BUFFER_LENGTH, &bytes_returned) == G_NO_ERROR){
			char * pch;
			pch = strtok(buffer,",");
			for (std::size_t i=0; i<info_.joints.size(); i++){
				if (pch != NULL){
					hw_states_velocity_[i] = atof( pch )/gears_m_2_cnt[i];
					pch = strtok (NULL, ",");
				}
			}
		}
		else {
			RCLCPP_ERROR(rclcpp::get_logger("GalilSystemHardwareInterface"), "Failed to send TV command to Galil.");
			return hardware_interface::return_type::ERROR;
		}
		return hardware_interface::return_type::OK;
  	}
  
  	hardware_interface::return_type GalilSystemHardwareInterface::write(const rclcpp::Time& /*time*/,
								      									const rclcpp::Duration& period){

		char command[1024]="";
		GSize BUFFER_LENGTH=1024;
		GSize bytes_returned;
		char buffer[1024];

		/*
		if( cmd_mode_ == 1 )
		{ sprintf( command, "PA " ); }		
		if( cmd_mode_ == 2 )
		{ sprintf( command, "PA " ); } // for position tracking mode
		*/ 
		//Mariana: Jazzy fix:
		if (cmd_mode_ == 1 || cmd_mode_ == 2){
			snprintf(command, sizeof(command), "PA ");
		}

		for (std::size_t i=0; i<info_.joints.size(); i++ ){
			char separator=',';
			if (i == info_.joints.size()-1)
				separator=' ';
			// The following blocks assume that i follows ABCD
			if (cmd_mode_ == 1){
				if (!isnan(hw_commands_position_[i]) && 0<strlen(command)){ 
					// Position Absolute command
					//sprintf(command, "%s%d%c", command, ((int)(hw_commands_position_[i]*gears_m_2_cnt[i])), separator); 
					// Mariana - Jazzy fix:
					int value = (int)(hw_commands_position_[i] * gears_m_2_cnt[i]);
					std::size_t len = std::strlen(command);
					std::snprintf(command + len, sizeof(command) - len, "%d%c", value, separator);
				}
			}
			// velocity command (no stopping)
			if (cmd_mode_ == 2){
				if (!isnan(hw_commands_velocity_[i]) && 0<strlen(command) ){
					if (i == 0 || i == 2){ 
						hw_commands_position_[i] += (period.seconds() * hw_commands_velocity_[i]/3.0);
					}
					else {
						hw_commands_position_[i] += (period.seconds() * hw_commands_velocity_[i]);
					}
					if (i == 0 && hw_commands_position_[i] < -0.03)
						hw_commands_position_[i] = -0.03;
					if (i == 0 && 0.03 <= hw_commands_position_[i])
						hw_commands_position_[i] = 0.03;	  
					if (i == 2 && hw_commands_position_[i] < -0.025)
						hw_commands_position_[i] = -0.025;
					if (i == 2 && 0.025 <= hw_commands_position_[i])
						hw_commands_position_[i] = 0.025;
					//sprintf( command, "%s%d%c", command, ((int)(hw_commands_position_[i]*gears_m_2_cnt[i])), separator );
					// Mariana - Jazzy fix:
					int value = (int)(hw_commands_position_[i] * gears_m_2_cnt[i]);
					std::size_t len = std::strlen(command);
					std::snprintf(command + len, sizeof(command) - len, "%d%c", value, separator);
				}
			}
    	}

		if (0<strlen(command)){
			std::cout << command << std::endl;
			int error = GCommand(connection, command, buffer, BUFFER_LENGTH, &bytes_returned );
			// don't send BG for PT
			if(error == G_NO_ERROR){
				// only send in position mode
				/*
				if( cmd_mode_ == 1 ){
				error = GCommand(connection, "BG", buffer, BUFFER_LENGTH, &bytes_returned );
				if( error == G_NO_ERROR ){}
				else{
					RCLCPP_ERROR(rclcpp::get_logger("GalilSystemHardwareInterface"), "Failed to send command: %s error code %d",
						command, error);
					error = GCommand(connection, "TC1", buffer, BUFFER_LENGTH, &bytes_returned );
					RCLCPP_ERROR(rclcpp::get_logger("GalilSystemHardwareInterface"), "TC1: %s.", buffer);
					return hardware_interface::return_type::ERROR;
				}
				}
				*/
			}
			else {
				RCLCPP_ERROR(rclcpp::get_logger("GalilSystemHardwareInterface"), "Failed to send command: %s error code %d",
							command, error);
				error = GCommand(connection, "TC1", buffer, BUFFER_LENGTH, &bytes_returned );
				RCLCPP_ERROR(rclcpp::get_logger("GalilSystemHardwareInterface"), "TC1: %s.", buffer);
				return hardware_interface::return_type::ERROR;
			}
		}
		else {
			// empty command
			/*RCLCPP_ERROR(rclcpp::get_logger("GalilSystemHardwareInterface"), "NaN command. Not sending.");*/
		}
		return hardware_interface::return_type::OK;
  	}
  
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(galil_driver::GalilSystemHardwareInterface, hardware_interface::SystemInterface)
