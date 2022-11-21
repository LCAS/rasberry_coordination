def debug_stagedecorator():
    registry = {}
    def registrar(cls):
        print("\n-----")
        print(cls.__name__)
        print(cls.__doc__)
        print("")
        print(cls.__init__.__name__)
        print(cls.__init__.__doc__)
        print("")
        print(cls._query.__name__)
        print(cls._query.__doc__)
        print("\n\n")
        return cls
    return registrar
def stagedecorator():
    registry = {}
    def registrar(cls):
        registry[cls.__name__] = cls
        return cls
    registrar.all = registry
    return registrar
_stagedecorator = stagedecorator()



def taskdecorator():
    registry = {}
    def registrar(cls):
        registry[cls.__name__] = cls
        return cls
    registrar.all = registry
    return registrar
_taskdecorator = taskdecorator()




@_stagedecorator
class stage_name():
    """This class is used for a, b, c"""
    def __init__():
        """This function is used to initialise stuff"""
        a=True
    def _query(self):
        """This function queries success?"""
        print("query passed")

@_stagedecorator
class stage_name_2(stage_name):
    """This class is used for a, b, c"""
    def __init__():
        """This function is used to initialise stuff"""
        a=True



print("--------")
print(_stagedecorator.all)
