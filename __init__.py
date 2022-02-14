try:
    import drake_jaco_driver.lcmtypes
    __path__.append(list(drake_jaco_driver.lcmtypes.__path__)[0] + "/drake_jaco_driver")
    from drake_jaco_driver.lcmtypes.drake_jaco_driver import *
except ImportError:
    pass
