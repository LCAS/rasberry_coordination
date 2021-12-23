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


def def_tasks(clean_task_list):
    from rasberry_coordination.coordinator_tools import logmsg
    logmsg(category="null")

    task_list = ['rasberry_coordination.task_management.custom_tasks.%s' % task for task in clean_task_list]
    if 'rasberry_coordination.task_management.base' not in task_list:
        task_list.insert(0, 'rasberry_coordination.task_management.base')

    logmsg(category="START",  msg="Collecting Interface, Task, and Stage Definitions for modules: ")
    [logmsg(category="START", msg="    | %s" % module) for module in clean_task_list]

    # For each task, import the TaskDef, StageDef, and InterfaceDef to the respective dictionaries
    task_definitions = dict()
    stage_definitions = dict()
    interface_definitions = dict()
    for task in task_list:
        task_definitions[task] = __import__('%s' % task, globals(), locals(), ['TaskDef'], -1).TaskDef
        stage_definitions[task] = __import__('%s' % task, globals(), locals(), ['StageDef'], -1).StageDef
        interface_definitions[task] = __import__('%s' % task, globals(), locals(), ['InterfaceDef'], -1).InterfaceDef

    # Make these accessible through import
    global TaskDef, StageDef, InterfaceDef
    TaskDef = type('TaskDef', tuple(task_definitions.values()), dict())
    StageDef = type('StageDef', tuple(stage_definitions.values()), dict())
    InterfaceDef = type('InterfaceDef', tuple(interface_definitions.values()), dict())

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
        NavigateToBaseNode (NavigateToNode[.], Navigation[:], StageBase[.:], object[::])
        ----------------
            DESC: Used for navigating to a given node
            __repr__
                | This is used to ...
            .   | Overridden to ...
            -> __init__
                | This is used to ...
            -> _start
                | This is used to ...
            -> _query
                | Completed on start
            :   | Overridden to complete on agent node bain the same as the target node
            -> _end
                | This is used to ...
            .   | This is used to ...
            .:  | This is used to ...
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
    from rasberry_coordination.task_management import Stg
    from rasberry_coordination.task_management import base
    from rasberry_coordination.task_management.custom_tasks import health_monitoring
    from rasberry_coordination.task_management.custom_tasks import transportation
    from rasberry_coordination.task_management.custom_tasks import uv_treatment
    from rasberry_coordination.task_management.custom_tasks import data_monitoring

    Stg(base.StageDef.__dict__['NavigateToBaseNode'])

    ba = [v for k, v in base.StageDef.__dict__.items() if not k.startswith('_')]
    hm = [v for k, v in health_monitoring.StageDef.__dict__.items() if not k.startswith('_')]
    tp = [v for k, v in transportation.StageDef.__dict__.items() if not k.startswith('_')]
    uv = [v for k, v in uv_treatment.StageDef.__dict__.items() if not k.startswith('_')]
    dm = [v for k, v in data_monitoring.StageDef.__dict__.items() if not k.startswith('_')]

    for v in ba + hm + tp + uv + dm:
        print(Stg(v, True))

# from rasberry_coordination.task_management import print_stages_md; print_stages_md()
