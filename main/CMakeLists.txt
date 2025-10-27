idf_component_register(
    SRCS "midi_host_example_main_txrx.c"
          REQUIRES driver
    INCLUDE_DIRS "."    
    PRIV_REQUIRES usb   
)