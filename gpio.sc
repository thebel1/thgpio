# Driver definition for native_ahci driver.
#
# When developing a driver for release through the async program:
#  * set "vendor" to the name of your company
#  * set "license" to one of the VMK_MODULE_LICENSE_* strings if applicable;
#    otherwise, set it to a 1-word identification of your module's license
#  * set "vendor_code" to your company's VMware-assigned Vendor Code
#  * set "vendor_email" to the contact e-mail provided by your company
#  * increment the version number if the source has come from VMware
#  * remove "version_bump" if present
#
# When bringing an async driver inbox at VMware:
#  * leave "version" as is from the async release
#  * set "license" to VMK_MODULE_LICENSE_BSD (unquoted)
#  * set "version_bump" to 1
#  * set "vendor" to "VMware, Inc."
#  * set "vendor_code" to "VMW"
#  * set "vendor_email" to the VMware contact e-mail address
#
# If updating the driver at VMware:
#  * increment "version bump" or contact the IHV for a new version number
#
# If updating the driver at an async vendor:
#  * increment the version number (do not touch version bump)

################
# NOTE: when making changes to to this file, please also
# update native_ahci.json.  Thanks, xxxxxxxxxxxxxxxxxxxxxx.
################

#
# Driver identification section
#
gpio_identification = {
   "name"               : "thgpio",
   "module type"        : "device driver",
   "binary compat"      : "yes",
   "summary"            : "Native GPIO Driver",
   "description"        : "Native GPIO Driver module for vmkernel",
   "version"            : "0.0.1",
   "license"            : "ThirdParty:MIT",
   "vendor"             : "Tom Hebel",
   "vendor_code"        : "THX",
   "vendor_email"       : "thebel@vmware.com",
   "componentName"      : "thgpio",
   "componentUIName"    : "Native GPIO Driver for vmkernel",
   "componentVersion"   : "1.0-0.0.0008",
   "componentUIVersion" : "0.0.1",
}

#
# Build the Driver Module
#
module_def = {
   "identification"  : gpio_identification,
   "source files"    : [ "gpio_os.c",
                         "gpio_drv.c",
                         "gpio_debug.c",
                       ],
}
gpio_module = defineKernelModule(module_def)

acpi_device_def = {
   "identification"  : gpio_identification,
   "device spec"     : "gpio_acpi_device.py",
}
gpio_acpi_devicek_def = defineDeviceSpec(acpi_device_def)

#
# Build the VIB
#
gpio_vib_def = {
   "identification"  : gpio_identification,
   "payload"         : [
                        gpio_module,
                        gpio_acpi_devicek_def
                       ],
   "vib properties"  : {
      "urls"                    : [ ],
      "provides"                : [ ],
      "depends"                 : [ ],
      "conflicts"               : [ ],
      "maintenance-mode"        : False,
      "hwplatform"              : [ ],
      "acceptance-level"        : 'community',
      "live-install-allowed"    : False,
      "live-remove-allowed"     : 'false',
      "cimom-restart"           : False,
      "stateless-ready"         : 'True',
      "overlay"                 : False,
   }
}
gpio_vib =  defineModuleVib(gpio_vib_def)

#
# Build the Component
#
gpio_bulletin_def = {
   "identification"  : gpio_identification,
   "vib"             : gpio_vib,
   "bulletin"        : {
      'kbUrl'       : 'http://kb.example.com/kb/example.html',
      'platforms'   : [
                        {'productLineID':'ESXi'},
                      ],
   },
}
gpio_bundle =  defineComponent(gpio_bulletin_def)
