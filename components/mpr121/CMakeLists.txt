set(component_srcs "mpr121.c")

# get IDF version for comparison
set(idf_version "${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}")

if(idf_version VERSION_GREATER_EQUAL "5.2")
	if(CONFIG_LEGACY_DRIVER)
		list(APPEND component_srcs "i2c_legacy.c")
	else()
		list(APPEND component_srcs "i2c_new.c")
	endif()
else()
	list(APPEND component_srcs "i2c_legacy.c")
endif()

idf_component_register(SRCS "${component_srcs}" PRIV_REQUIRES driver INCLUDE_DIRS ".")
