idf_component_register(SRCS "ulp_test_readings.c"
                       INCLUDE_DIRS ""
                       REQUIRES soc nvs_flash ulp driver)

# ULP support additions to component CMakeLists.txt.
#
# 1. The ULP app name must be "ulp_main"
set(ulp_app_name ulp_main)
#
# 2. Specify all assembly source files.
#    Paths are relative because ULP files are placed into a special directory "ulp"
#    in the root of the project
set(ulp_s_sources "../ulp/adc.S")
#
# 3. List all the component source files which include automatically
#    generated ULP export file, ${ulp_app_name}.h:
set(ulp_exp_dep_srcs "ulp_test_readings.c")
#
# 4. Call function to build ULP binary and embed in project using the argument
#    values above.
ulp_embed_binary(${ulp_app_name} ${ulp_s_sources} ${ulp_exp_dep_srcs})
