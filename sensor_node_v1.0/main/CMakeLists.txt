# See the build system documentation in IDF programming guide
# for more information about component CMakeLists.txt files.

idf_component_register(SRCS "main.c"
						    "canbus.c"
						    "i2c_bus.c"
						    "spi_bus.c"
                            "modbus.c"
                    INCLUDE_DIRS ".")
