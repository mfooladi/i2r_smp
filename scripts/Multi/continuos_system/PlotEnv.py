import vector_fild_new as vf

agent = vf.Robot(geometry_name=vf.r1_geometry, agent_radius=100, agent_id=1, v_max= .5 , goals_file=vf.r1_goals, init_pos=[0.,0.])\

agent.plot_triangles()