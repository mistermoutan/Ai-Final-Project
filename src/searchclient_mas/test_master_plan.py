from master_plan import MasterPlan
import test_utilities as util


agt0 = util.agent(0,"red")
agt1 = util.agent(1,"blue")
agt2 = util.agent(2, "red")
box0 = util.box("A", "red")

level = [
    [False,False,False,False,False],
    [False,agt0,True,agt2,False],
    [False,True,True,True,False],
    [False,box0,True,agt1,False],
    [False,False,False,False,False]
    ]
initial_state = util.make_state(level)

#def test_merge_plan_into_emtpy_master_works():
    #p = MasterPlan(3, initial_state)
    #p.merge_plan_into_master(0)