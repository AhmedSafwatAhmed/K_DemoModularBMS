#!/usr/bin/env python3
"""
EV-BMS Project Structure Creator
This script creates the complete folder structure and placeholder files
for the Battery Management System project.
"""

import os

# Define project structure
project_structure = {
    "EV-BMS": {
        "Config": ["bms_config.h", "board_config.h"],
        "MCAL": [
            "spi.h", "spi.c",
            "uart.h", "uart.c", 
            "pwm.h", "pwm.c",
            "gpio.h", "gpio.c",
            "timer.h", "timer.c",
            "i2c.h", "i2c.c"
        ],
        "HAL": [
            "slave_ctrl_if.h", "slave_ctrl_if.c",
            "lcd_if.h", "lcd_if.c"
        ],
        "Services": [
            "debug_info_mgr.h", "debug_info_mgr.c",
            "bms_database.h", "bms_database.c",
            "wdg_mgr.h", "wdg_mgr.c"
        ],
        "Application": [
            "cell_balancing_mgr.h", "cell_balancing_mgr.c",
            "diagnostics_mgr.h", "diagnostics_mgr.c",
            "fusa_supervisor.h", "fusa_supervisor.c",
            "battery_status_mon.h", "battery_status_mon.c",
            "fan_control.h", "fan_control.c",
            "thermal_mgr.h", "thermal_mgr.c"
        ],
        "Docs": ["BMS_System_Documentation.md", "Architecture_Diagram.txt"],
        "": ["main.c", "README.md", ".gitignore", "Makefile"]
    }
}

def create_project_structure(base_path="."):
    """Create all directories and empty files"""
    
    for project_name, structure in project_structure.items():
        project_path = os.path.join(base_path, project_name)
        
        print(f"Creating project: {project_name}")
        
        for folder, files in structure.items():
            if folder:  # If folder name is not empty
                folder_path = os.path.join(project_path, folder)
                os.makedirs(folder_path, exist_ok=True)
                print(f"  Created folder: {folder}")
                
                for file in files:
                    file_path = os.path.join(folder_path, file)
                    if not os.path.exists(file_path):
                        with open(file_path, 'w') as f:
                            f.write(f"/* {file} - To be implemented */\n")
                        print(f"    Created file: {file}")
            else:  # Root level files
                for file in files:
                    file_path = os.path.join(project_path, file)
                    if not os.path.exists(file_path):
                        with open(file_path, 'w') as f:
                            if file == ".gitignore":
                                f.write(create_gitignore())
                            elif file == "Makefile":
                                f.write(create_makefile())
                            else:
                                f.write(f"/* {file} - To be implemented */\n")
                        print(f"  Created file: {file}")
    
    print("\n✅ Project structure created successfully!")
    print(f"\nNext steps:")
    print(f"1. cd {project_name}")
    print(f"2. code .  (opens VS Code)")
    print(f"3. Copy the code from the documentation into each file")

def create_gitignore():
    """Generate .gitignore content"""
    return """# Build artifacts
*.o
*.elf
*.bin
*.hex
*.map
*.lst

# IDE files
.vscode/
*.uvoptx
*.uvguix
*.bak

# Debug files
*.log
Debug/
Release/

# OS files
.DS_Store
Thumbs.db
"""

def create_makefile():
    """Generate basic Makefile"""
    return """# EV-BMS Makefile
CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy

# Compiler flags
CFLAGS = -mcpu=cortex-m0plus -mthumb -O2 -g -Wall
CFLAGS += -DMKL25Z128VLK4

# Include paths
INCLUDES = -I./Config -I./MCAL -I./HAL -I./Services -I./Application

# Source files
SRCS = main.c \\
       MCAL/spi.c MCAL/uart.c MCAL/pwm.c MCAL/gpio.c MCAL/timer.c MCAL/i2c.c \\
       HAL/slave_ctrl_if.c HAL/lcd_if.c \\
       Services/debug_info_mgr.c Services/bms_database.c Services/wdg_mgr.c \\
       Application/diagnostics_mgr.c Application/fusa_supervisor.c \\
       Application/battery_status_mon.c Application/fan_control.c \\
       Application/thermal_mgr.c Application/cell_balancing_mgr.c

OBJS = $(SRCS:.c=.o)

# Target
TARGET = bms

all: $(TARGET).bin

$(TARGET).elf: $(OBJS)
\t$(CC) $(CFLAGS) $(OBJS) -o $@

$(TARGET).bin: $(TARGET).elf
\t$(OBJCOPY) -O binary $< $@

%.o: %.c
\t$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

clean:
\t rm -f $(OBJS) $(TARGET).elf $(TARGET).bin

.PHONY: all clean
"""

if __name__ == "__main__":
    create_project_structure()