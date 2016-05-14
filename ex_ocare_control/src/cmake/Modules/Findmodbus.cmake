find_package(PkgConfig)
pkg_check_modules(PC_MODBUS QUIET libmodbus)
set(MODBUS_DEFINITIONS ${PC_MODUBS_CFLAGS_OTHER})



find_path(MODBUS_INCLUDE_DIR modbus/modbus.h
          HINTS ${PC_MODBUS_INCLUDEDIR} ${PC_MODBUS_INCLUDE_DIRS}
          PATH_SUFFIXES libxml2 )

find_library(MODBUS_LIBRARY NAMES modbus
             HINTS ${PC_MODBUS_LIBDIR} ${PC_MODBUS_LIBRARY_DIRS} )

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Modbus  DEFAULT_MSG
                                  MODBUS_LIBRARY MODBUS_INCLUDE_DIR)

mark_as_advanced(MODBUS_INCLUDE_DIR MODBUS_LIBRARY )

set(MODBUS_LIBRARIES ${MODBUS_LIBRARY} )
set(MODBUS_INCLUDE_DIRS ${MODBUS_INCLUDE_DIR} )
