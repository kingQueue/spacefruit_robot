function(generate_cfs_startup_script CFS_INSTALL_DIR)
    file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/${CFS_INSTALL_DIR}/cfe_es_startup.scr"
        "CFE_APP, robot_app, ROBOT_APP_Main, ROBOT_APP, 50, 32768, 0x0, 0;\n")
endfunction()
