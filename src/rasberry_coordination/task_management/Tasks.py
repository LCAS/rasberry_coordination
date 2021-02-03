class TaskDef(object):
    """ Definitions for Task Initialisation Criteria """
    # Consistent attributes accessible directly, task-specific
    # attributes accessed through details dict defined on task
    # stage start.
    # Task definitions are defined in the perspective of the
    # agent handling the task.


    """ Runtsime Method for Custom Task Definitions """
    @classmethod
    def generate_task(cls, agent, list, details={}):
        agent.task_details = details
        agent.task_stage_list = []

        #Create dictionary for access to each stage defined in StageDef
        stage_dict = {stage:StageDef().__getattribute__(stage)
                      for stage in dir(StageDef)
                      if not stage.startswith('__')}

        #For each required stage, append the StageDef.Stage to a list
        for S in task_stage_list:
            if S == "start_task":
                agent.task_stage_list.append(stage_dict[S]())
            else:
                agent.task_stage_list.append(stage_dict[S](agent))


    """ Initial Task Stages for Agents """
    @classmethod
    def idle_picker(cls, agent, details={}):
        agent.task_details = details
        agent.task_stage_list.append(
            StageDef.IdlePicker(agent)
        )
    @classmethod
    def idle_courier(cls, agent, details={}):
        agent.task_details = details
        agent.task_stage_list.append(
            StageDef.IdleCourier(agent)
        )
    @classmethod
    def idle_storage(cls, agent, details={}):
        agent.task_details = details
        agent.task_stage_list.append(
            StageDef.IdleStorage(agent)
        )

    """ Picker Logistics Transportation """
    @classmethod
    def transportation_request(cls, agent, details={}):
        agent.task_details = details
        agent.task_stage_list.append(
            StageDef.StartTask(),
            StageDef.AssignCourier(agent),
            # robot has accepted task (ACCEPT)
            StageDef.AwaitCourier(agent),
            # robot has arrived (ARRIVED)
            StageDef.LoadCourier(agent),
            # task is marked as complete (INIT)
            StageDef.IdlePicker(agent) #not needed, added automatically when empty
        )
    @classmethod
    def transportation_courier(cls, agent, details={}):
        agent.task_details = details
        agent.task_stage_list.append(
            StageDef.StartTask(),
            StageDef.NavigateToPicker(agent),
            StageDef.Loading(agent),
            StageDef.AssignStorage(agent),
            StageDef.AssignWaitNode(agent),
            StageDef.AwaitStoreAccess(agent),
            StageDef.NavigateToStorage(agent),
            StageDef.Unloading(agent),
            StageDef.IdleCourier(agent)
        )
    @classmethod
    def transportation_storage(cls, agent, details={}):
        agent.task_details = details
        agent.task_stage_list.append(
            StageDef.StartTask(),
            StageDef.AwaitCourier(agent),
            StageDef.UnloadCourier(agent),
            StageDef.AwaitCourierExit(agent),
            StageDef.IdleStorage(agent)
        )

    """ Trailer Logistics Transportation """
    @classmethod
    def trailer_courier(cls, agent, details={}):
        agent.task_details = details
        agent.task_stage_list.append(
            StageDef.AssignStorage(agent),
            StageDef.AwaitStoreAccess(agent),
            StageDef.NavigateToStorage(agent),
            StageDef.Loading(agent),
            StageDef.AssignStorage(agent),
            StageDef.AssignWaitNode(agent),
            StageDef.AwaitStoreAccess(agent),
            StageDef.NavigateToStorage(agent),
            StageDef.Loading(agent),
            StageDef.IdleCourier(agent)
        )

    """ UV Treatment """
    @classmethod
    def uv_edge_treatment(cls, agent, details={}):
        agent.task_stage_list.append(
            ("navigation",    StageDef.Navigation(agent, details['edge_start_node'])),
            ("uv_treat_edge", StageDef.EdgeUVTreatment(agent, details['edge_end_node']))
        )
    @classmethod
    def uv_row_treatment(cls, agent, details={}):
        agent.task_stage_list.append(
            ("navigation",    StageDef.Navigation(agent, details['row_start_node'])),
            ("uv_treat_row",  StageDef.RowUVTreatment(agent, details['row_end_node']))
        )

    """ Monitoring """
    @classmethod
    def data_collection_edge(cls, agent, details={}):
        agent.task_stage_list.append(
            ("navigation",    StageDef.Navigation(agent, details['row_start_node'])),
            ("data_collection",  StageDef.EdgeDataCollection(agent, details['row_end_node']))
        )
    @classmethod
    def data_collection_row(cls, agent, details={}):
        agent.task_stage_list.append(
            ("navigation", StageDef.Navigation(agent, details['row_start_node'])),
            ("data_collection", StageDef.RowDataCollection(agent, details['row_end_node']))
        )
