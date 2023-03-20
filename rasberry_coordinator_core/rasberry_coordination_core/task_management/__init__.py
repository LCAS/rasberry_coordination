#Builtins
from os import walk
import os.path, pkgutil
import importlib
from pprint import pprint
import traceback

# Components
global Stages, Interfaces, PropertiesDef
Stages, Interfaces, PropertiesDef = None, None, None

# Logging
from rasberry_coordination_core.utils.logmsg import logmsg
from rasberry_coordination_core.node import GlobalNode


def get_package_host(pkg_name):
    if 'rasberry_coordination_core' in pkg_name:
       host = pkg_name.split('.')[3]
    else:
       host = pkg_name.split('.')[0]
    return host


def set_properties(package_dict):
    global PropertiesDef

    PropertiesDef = {M['name']: M['properties'] for M in package_dict if 'properties' in M}
    PropertiesDef.update({'base': {}})
    namespace = '/rasberry_coordination/task_modules/%s/%s'

    logmsg(category="START",  msg="Properties: ")
    for M in package_dict:
        logmsg(category="START",  msg="   | %s" % get_package_host(M['name']))
        try:
            if 'properties' in M:
                [logmsg(category="START", msg="   :   | %s -> %s" % (key, val)) for key, val in M['properties'].items()]
                [GlobalNode.declare_paramater(namespace % (M['name'], k), v) for k, v in M['properties'].items()]
        except:
           print(traceback.format_exc())


def fetch_property(module, key, default=None):
    return GlobalNode.get_parameter_or('/rasberry_coordination/task_modules/%s/%s' % (module, key), default)


def load_custom_modules(clean_package_list):
    import rasberry_coordination_core
    base_folder = rasberry_coordination_core.__path__[-1] + "/task_management/modules"
    std_packages = next(walk(base_folder), (None, None, []))[1]

    clean_package_list = [t for t in clean_package_list if t not in std_packages]
    package_list = ['%s.coordination.task_module' % package for package in clean_package_list]
    #^ change to import the three from seperate files

    package_list.insert(0, 'rasberry_coordination_core.task_management.modules.base')
    package_list.insert(1, 'rasberry_coordination_core.task_management.modules.navigation')
    package_list.insert(2, 'rasberry_coordination_core.task_management.modules.assignment')

    logmsg(category="START",  msg="Collecting Interfaces and Stage Definitions for Active Packages: ")
    [logmsg(category="START", msg="   | %s" % package) for package in package_list]

    # For each package, import the Stages, and Interfaces to their respective containers
    # these containers can then be imported directly to access all components irrespective of location
    # thus allowing for the coordinator to not be modified when new modules are added within packages
    global Stages, Interfaces


    # Import Stages
    logmsg(category="START", msg="STAGES: ")
    from rasberry_coordination_core.task_management.modules.base.stage_definitions import StageBase

    # Loop through list of included packages
    Stages = dict()
    for package in package_list:

        # Identify the host to reference by
        host = get_package_host(package)
        Stages[host] = dict()
        logmsg(category="START", msg="   | %s" % host.upper())
        pkg = f"{package}.stage_definitions"

        # Attempt to load the package
        try:
            stages_module = importlib.import_module(pkg)
        except ImportError as e:
            url = "https://github.com/Iranaphor/rasberry-coordination/wiki/ImportError-Debugging"
            logmsg(level="error", category="START", msg=f"   :   | When importing: {host}.stage_definitions")
            logmsg(level="error", category="START", msg=f"   :   :   | {str(e).split(' (')[0]}")
            logmsg(level="error", category="START", msg=f"   :   :   | TRY: python3 -c 'import {pkg}.stage_definitions'")
            #logmsg(level="error",category="START", msg=f"   :   :   | FAQ: {url}")
            print(traceback.format_exc())
            exit()


        # Add the iFACE for each file
        for stage_class_name in dir(stages_module):
            try:
                stage_class = getattr(stages_module, stage_class_name)
                if type(stage_class)!=type(StageBase) or not issubclass(stage_class, StageBase):
                    continue
                logmsg(category="START", msg=f"   :   | {stage_class_name}")
                Stages[host][stage_class_name] = stage_class
            except ImportError as e:
                url = "https://github.com/Iranaphor/rasberry-coordination/wiki/ImportError-Debugging"
                logmsg(level="error", category="START", msg=f"   :   | When importing: {host}.stage_definitions.{stage_class_name}")
                logmsg(level="error", category="START", msg=f"   :   :   | {str(e).split(' (')[0]}")
                logmsg(level="error", category="START", msg=f"   :   :   | TRY: python3 -c 'import {pkg}.stage_definitions'")
                #logmsg(level="error",category="START", msg=f"   :   :   | FAQ: {url}")
                print(traceback.format_exc())
                exit()

    # Import Interfaces
    logmsg(category="START", msg="INTERFACES: ")
    from rasberry_coordination_core.task_management.modules.base.interfaces.Interface import iFACE as InterfaceBase


    # Loop through list of included packages
    Interfaces = dict()
    for package in package_list:

        # Identify the route for the interface sub-package
        host = get_package_host(package)
        Interfaces[host] = dict()
        logmsg(category="START", msg="   | %s" % host.upper())
        pkg = f"{package}.interfaces"


        # Attempt to load the package and identify all of its modules
        try:
            interface_pkg = importlib.import_module(pkg)
            interface_pkg_path = os.path.dirname(interface_pkg.__file__)
            interface_modules = [name for _, name, _ in pkgutil.iter_modules([interface_pkg_path])]
        except ImportError as e:
            url = "https://github.com/Iranaphor/rasberry-coordination/wiki/ImportError-Debugging"
            logmsg(level="error", category="START",msg=f"   :   | {package} ({str(e)})")
            print(traceback.format_exc())
            logmsg(level="error", category="START", msg="   :   | Ensure <<package>>.interfaces.<<module>>.iFACE")
            logmsg(level="error", category="START",msg=f"   :   | FAQ @ {url}")
            exit()

        # Add the iFACE for each file
        for interface_module in interface_modules:
            try:
                iFACE = importlib.import_module(f"{pkg}.{interface_module}").iFACE
                if type(iFACE) != type(InterfaceBase) or not issubclass(iFACE, InterfaceBase):
                    logmsg(level="error", category="START", msg=f"   :   | {submodule} invalid")
                    exit()
                logmsg(category="START", msg=f"   :   | {interface_module}")
                Interfaces[host][interface_module] = iFACE
            except ImportError as e:
                url = "https://github.com/Iranaphor/rasberry-coordination/wiki/ImportError-Debugging"
                logmsg(level="error", category="START", msg=f"   :   | When importing: {host}.interfaces.{interface_module}")
                logmsg(level="error", category="START", msg=f"   :   :   | {str(e).split(' (')[0]}")
                logmsg(level="error", category="START", msg=f"   :   :   | TRY: python3 -c 'import {pkg}.{interface_module}'")
                #logmsg(level="error",category="START", msg=f"   :   :   | FAQ: {url}")
                print(traceback.format_exc())
                exit()
