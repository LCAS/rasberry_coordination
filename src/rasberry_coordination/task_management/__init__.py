from rospy import set_param, get_param
from rasberry_coordination.coordinator_tools import logmsg

global TaskDef, StageDef, InterfaceDef, PropertiesDef


def set_properties(module_dict):
    global PropertiesDef
    PropertiesDef = {M['name']: M['properties'] for M in module_dict}
    PropertiesDef.update({'base': {}})
    namespace = '/rasberry_coordination/task_modules/%s/%s'

    logmsg(category="START",  msg="Properties: ")
    for M in module_dict:
        logmsg(category="START",  msg="    - %s" % M['name'])
        [logmsg(category="START", msg="        | %s -> %s" % (key, val)) for key, val in M['properties'].items()]
        [set_param(namespace % (M['name'], k), v) for k, v in M['properties'].items()]


def fetch_property(module, key, default=None):
    return get_param('/rasberry_coordination/task_modules/%s/%s' % (module, key), default)

def rename(cls, prefix):
    for fcn_name in [f for f in dir(cls) if not f.startswith('__') and prefix != 'base']:
        fcn = getattr(cls, fcn_name)
        delattr(cls, fcn_name)
        setattr(cls, prefix+'_'+fcn_name, fcn)
    return cls

def load_custom_modules(clean_module_list):
    from rasberry_coordination.coordinator_tools import logmsg
    logmsg(category="null")

    clean_module_list = [t for t in clean_module_list if t != 'base']
    module_list = ['rasberry_coordination.task_management.custom_tasks.%s' % module for module in clean_module_list]

    if 'rasberry_coordination.task_management.base' not in module_list:
        module_list.insert(0, 'rasberry_coordination.task_management.base')

    logmsg(category="START",  msg="Collecting Interface, Task, and Stage Definitions for modules: ")
    [logmsg(category="START", msg="    | %s" % module) for module in clean_module_list]

    # For each module, import the TaskDef, StageDef, and InterfaceDef to their respective containers
    interface_definitions = dict()
    task_definitions = dict()
    stage_definitions = dict()
    for module in module_list:
        module_class = __import__(module, globals(), locals(), ['TaskDef','StageDef','InterfaceDef'], -1)
        interface_definitions[module] = rename(module_class.InterfaceDef, module.split('.')[-1])
        task_definitions[module] = module_class.TaskDef
        stage_definitions[module] = module_class.StageDef

    # Compile containers down to single global objects to import from within rasberry_coordination
    global TaskDef, StageDef, InterfaceDef
    InterfaceDef = type('InterfaceDef', tuple(interface_definitions.values()), dict())
    TaskDef = type('TaskDef', tuple(task_definitions.values()), dict())
    StageDef = type('StageDef', tuple(stage_definitions.values()), dict())

    logmsg(category="START",  msg="Interfaces: ")
    [logmsg(category="START", msg="    | %s" % i) for i in dir(InterfaceDef) if not i.startswith('__')]
    logmsg(category="START",  msg="Tasks: ")
    [logmsg(category="START", msg="    | %s" % t) for t in dir(TaskDef) if not t.startswith('__')]
    logmsg(category="START",  msg="Stages: ")
    [logmsg(category="START", msg="    | %s" % s) for s in dir(StageDef) if not s.startswith('__')]


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
    from rasberry_coordination.task_management.custom_tasks import health_monitoring
    from rasberry_coordination.task_management.custom_tasks import transportation
    from rasberry_coordination.task_management.custom_tasks import uv_treatment
    from rasberry_coordination.task_management.custom_tasks import data_monitoring

    # Stg(base.StageDef.__dict__['NavigateToBaseNode'])
    ba = ([v for k, v in base.StageDef.__dict__.items() if not k.startswith('_')],              base)
    hm = ([v for k, v in health_monitoring.StageDef.__dict__.items() if not k.startswith('_')], health_monitoring)
    tp = ([v for k, v in transportation.StageDef.__dict__.items() if not k.startswith('_')],    transportation)
    uv = ([v for k, v in uv_treatment.StageDef.__dict__.items() if not k.startswith('_')],      uv_treatment)
    dm = ([v for k, v in data_monitoring.StageDef.__dict__.items() if not k.startswith('_')],   data_monitoring)

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