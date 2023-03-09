from rasberry_coordination_core.logmsg_utils import logmsg

class ModuleObj(object):

    def __repr__(self):
        return "Module( name:%s | interface:%s )" % (self.name, self.interface or None)

    def __init__(self, agent, name, interface, details):
        logmsg(category="module", msg="   | %s (%s)"%(name.upper(),interface.upper()))
        self.agent = agent
        self.name = name
        from rasberry_coordination_core.task_management.__init__ import Interfaces, PropertiesDef
        self.interface = Interfaces[name][interface](agent=agent, details=details)
        self.properties = PropertiesDef[name] if name in PropertiesDef else dict()
        self.details = details

    def add_init_task(self):
        logmsg(category="module", msg="%s" % self.name.upper())
        logmsg(category="module", msg="   | Interface: %s" % self.interface)
        logmsg(category="module", msg="   | Searching for init task:")
        self.agent.add_task(module=self.name, name='init')

