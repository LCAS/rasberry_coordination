# if __name__ == '__main__':
#     from pprint import pprint
#
#     def filter(l):
#         return [f for f in dir(l) if not f.startswith('__')]
#
#     def rename(interface, prefix):
#         for fcn_name in filter(interface):
#             fcn = getattr(interface, fcn_name)
#             delattr(interface, fcn_name)
#             setattr(interface, prefix+fcn_name, fcn)
#         return interface
#
#
#     print('')
#     tp='rasberry_coordination.task_management.custom_tasks.transportation'
#     module_class = __import__(tp, globals(), locals(), ['TaskDef', 'StageDef', 'InterfaceDef'], -1)
#     ti = rename(interface=module_class.InterfaceDef, prefix='FP_')
#     print(filter(ti))
#
#     print('')
#     uv='rasberry_coordination.task_management.custom_tasks.uv_treatment'
#     module_class = __import__(uv, globals(), locals(), ['TaskDef', 'StageDef', 'InterfaceDef'], -1)
#     ui = rename(interface=module_class.InterfaceDef, prefix='UV_')
#     print(filter(ui))
#
#     print('\n')
#     Id2 = type('Id2', tuple([ti, ui]), dict())
#     pprint(filter(Id2))
#     print('created Id2 from `ti,ui`\n')
#
#
