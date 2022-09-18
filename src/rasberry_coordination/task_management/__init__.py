import importlib
from pprint import pprint
from rospy import set_param, get_param
from rasberry_coordination.coordinator_tools import logmsg

global Stages, Interfaces, PropertiesDef


def get_module_host(module):
    if 'rasberry_coordination' in module:
       host = module.split('.')[3]
    else:
       host = module.split('.')[0]
    return host


def set_properties(module_dict):
    global PropertiesDef
    PropertiesDef = {M['name']: M['properties'] for M in module_dict if 'properties' in M}
    PropertiesDef.update({'base': {}})
    namespace = '/rasberry_coordination/task_modules/%s/%s'

    logmsg(category="START",  msg="Properties: ")
    for M in module_dict:
        logmsg(category="START",  msg="    | %s" % get_module_host(M['name']))
        try:
            [logmsg(category="START", msg="    :    | %s -> %s" % (key, val)) for key, val in M['properties'].items()]
            [set_param(namespace % (M['name'], k), v) for k, v in M['properties'].items()]
        except:
           pass


def fetch_property(module, key, default=None):
    return get_param('/rasberry_coordination/task_modules/%s/%s' % (module, key), default)


def rename(cls, prefix):
    for fcn_name in [f for f in dir(cls) if not f.startswith('__') and prefix != 'base']:
        fcn = getattr(cls, fcn_name)
        delattr(cls, fcn_name)
        setattr(cls, prefix+'_'+fcn_name, fcn)
    return cls


def load_custom_modules(clean_module_list):
    clean_module_list = [t for t in clean_module_list if t != 'base']
    module_list = ['%s.coordination.task_module' % module for module in clean_module_list] #change to import the three from seperate files
    module_list.insert(0, 'rasberry_coordination.task_management.modules.base')
    module_list.insert(1, 'rasberry_coordination.task_management.modules.navigation')
    module_list.insert(2, 'rasberry_coordination.task_management.modules.assignment')

    logmsg(category="START",  msg="Collecting Interfaces and Stage Definitions for modules: ")
    [logmsg(category="START", msg="    | %s" % module) for module in module_list]


    # For each module, import the Stages, and Interfaces to their respective containers
    # these containers can then be imported directly to access all components irrespective of location
    # thus it allows for the coordinator to not be modified when new python classes are added for modules
    global Stages, Interfaces

    logmsg(category="START", msg="STAGES: ")
    from rasberry_coordination.task_management.modules.base.stage_definitions import StageBase
    Stages = dict()
    for module in module_list:
        host = get_module_host(module)
        Stages[host] = dict()
        logmsg(category="START", msg="    | %s" % host.upper())

        ### module_obj = __import__(module+".stage_definitions", globals(), locals(), [''], -1) #this looked cooler
        try:
            module_obj = importlib.import_module(module+".stage_definitions")
        except ImportError as e:
            print(e)
            continue

        for obj in dir(module_obj):
            cls = getattr(module_obj, obj)

            if type(cls)!=type(StageBase) or not issubclass(cls, StageBase): continue
            Stages[host][obj] = cls
            logmsg(category="START", msg="    :    | %s" % obj)

    # Import Interfaces
    logmsg(category="START", msg="INTERFACES: ")
    logmsg(category="START", msg="    | if an expected interface has not appeard")
    logmsg(category="START", msg="    | try importing directly in a python terminal")
    from rasberry_coordination.task_management.modules.base.interfaces.Interface import Interface as InterfaceBase
    Interfaces = dict()
    for module in module_list:
        host = get_module_host(module)
        Interfaces[host] = dict()
        logmsg(category="START", msg="    | %s" % host.upper())

        ### module_obj = __import__(module, globals(), locals(), ['interfaces'], -1).interfaces #this looked cooler
        try:
            module_obj = importlib.import_module(module+".interfaces")
        except ImportError as e:
            logmsg(category="START", msg="    :    | "+str(e))
            logmsg(category="START", msg="    :    | ensure ...interfaces.__init__.py imports to your modules")
            continue

        for obj in [d for d in dir(module_obj) if not d.startswith('__')]:
            file = getattr(module_obj, obj)

            if not hasattr(file, obj):
                continue
            cls = getattr(file, obj)

            if type(cls)!=type(InterfaceBase) or not issubclass(cls, InterfaceBase): continue
            Interfaces[host][obj] = cls
            logmsg(category="START", msg="    :    | %s" % obj)

class Stg(object):
    from math import ceil, floor

    def __init__(self, cls, md=False):
        self.functions = ['__repr__', '__init__', '_start', '_query', '_end']

        self.md = md
        self.md_surround = '`' if md else ''
        self.md_spacer = '\n' if self.md else ''

        self.name = cls.__name__

        self.parents = [cls]
        self.get_parents()

        self.indexes = []
        self.get_indexes()

    def get_parents(self):
        while self.parents[-1] is not object:
            self.parents.append(self.parents[-1].__bases__[0])
        self.parents = [p for p in self.parents[:-1]]

    def get_indexes(self):
        self.indexes = []
        for i in range(len(self.parents)):
            a, b = divmod(i, 2)
            self.indexes.append(('.' if b else '') + (':' * a))
        self.indexes = [' '*(a+b-len(i)) + i for i in self.indexes]

    def __repr__(self):
        """
        ---------------------------------------------------------------------------
        #### NavigateToBaseNode: [`NavigateToNode(.)`, `Navigation(:)`, `StageBase(.:)`]
        Used to navigate to a given base_node

            -> __repr__
               | .: | Simplified representation of class for clean informative logging
               |  : | Display class with idle navigation target

            -> __init__
               | .: | Class initialisation for populating default values
               |  : | Identify the location from which the target is identified
               |    | Call super to set association to base_node

            -> _start
               | .: | Stage start, called when this is the active stage.
               |  : | Flag the agent as requiring a route
               |  . | Start task by setting the target to a given node

            -> _query
               | .: | Used to define the criteria which must be met for the stage to be completed
               |  : | Complete when the agent's location is identical to the target location.

            -> _end
               | .: | Used to set any fields as the stage is about to be removed
               |  : | End navigation by refreshing routes for other agents in motion.
        """
        self.repr = ''
        def pint(s): self.repr = self.repr+"\n"+s
        pint("\n")
        z = zip(self.parents[1:], self.indexes[1:])
        parents = str(["%s(%s)" % (p.__name__, i.replace(' ', '')) for p, i in z]).replace("'", "%s" % self.md_surround)
        title = "%s: %s" % (self.parents[0].__name__, parents)
        if self.md:
            pint("-"*len(title))
            pint("#### %s" % title)
            pint("%s" % self.parents[0].__doc__)
        else:
            pint(title)
            pint("-"*len(title))
            pint("    DESC: %s" % self.parents[0].__doc__)

        for f in self.functions:
            self.repr += self.md_spacer
            pint("    -> %s" % f)
            zp = zip(self.parents, self.indexes)
            zp.reverse()
            for p, i in zp:
                if f in p.__dict__:
                    d = p.__dict__[f].__doc__
                    pint("       | %s | %s" % (i, d))
        pint("\n")
        return self.repr


def print_stages_md():
    import rasberry_coordination
    from rasberry_coordination.task_management import Stg
    from rasberry_coordination.task_management import base
    from rasberry_coordination.task_management.modules import health_monitoring
    from rasberry_coordination.task_management.modules import transportation
    from rasberry_coordination.task_management.modules import uv_treatment
    from rasberry_coordination.task_management.modules import data_collection

    # Stg(base.StageDef.__dict__['NavigateToBaseNode'])
    ba = ([v for k, v in base.StageDef.__dict__.items() if not k.startswith('_')],              base)
    hm = ([v for k, v in health_monitoring.StageDef.__dict__.items() if not k.startswith('_')], health_monitoring)
    tp = ([v for k, v in transportation.StageDef.__dict__.items() if not k.startswith('_')],    transportation)
    uv = ([v for k, v in uv_treatment.StageDef.__dict__.items() if not k.startswith('_')],      uv_treatment)
    dm = ([v for k, v in data_collection.StageDef.__dict__.items() if not k.startswith('_')],   data_collection)

    # import rospkg
    # rospack = rospkg.RosPack()
    # rospack.list()
    # rasberry_coordination = rospack.get_path('rasberry_coordination')
    filepath = rasberry_coordination.__path__[-1].replace('/src/rasberry_coordination', '/docs/StageList.md')
    modules = [ba, hm, tp, uv, dm]
    with open(filepath, 'w') as f:

        f.write('# Stage List:\nThis document details the structure and formatting of the stages defined in the StageDef for each module in:\n\n')

        for m in modules:
            f.write('\n- [%s](#%s)' % (m[1].__doc__, m[1].__doc__.replace(' ','')))

        for m in modules:
            module_name = m[1].__doc__
            module_ref = m[1].__name__.replace('rasberry_coordination.task_management.','')
            f.write('\n\n%s\n<a name="%s"></a>\n## %s\nStages defined in the %s module:  ' % ("-"*50, module_name.replace(' ', ''), module_name, module_ref))
            for v in m[0]: f.write(Stg(v, True).__repr__())

# if __name__ == '__main__':
    # from rasberry_coordination.task_management import print_stages_md; print_stages_md()
    # print_stages_md()
