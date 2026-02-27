set(component_srcs "mpr121.c")

# get IDF version for comparison
set(idf_version "${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}")

if(idf_version VERSION_LESS_EQUAL "5.2")
	list(APPEND component_srcs "i2c_legacy.c")
else()
	if(CONFIG_LEGACY_DRIVER)
		list(APPEND component_srcs "i2c_legacy.c")
	else()
		list(APPEND component_srcs "i2c_new.c")
	endif()
endif()

idf_component_register(
	SRCS "${component_srcs}"
	PRIV_REQUIRES driver esp_driver_i2c esp_driver_gpio
	INCLUDE_DIRS "."
)
