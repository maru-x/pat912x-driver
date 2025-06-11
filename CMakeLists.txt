zephyr_library()
zephyr_library_sources_ifdef(CONFIG_INPUT_PAT912X drivers/input/input_pat912x.c)
zephyr_include_directories(${APPLICATION_SOURCE_DIR}/include)
