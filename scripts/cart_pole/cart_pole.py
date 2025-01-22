import numpy as np
import os
import sys
import matplotlib.pyplot as plt
from termcolor import colored
import scipy.io
import time
import pdb

from pybullet_dynamics.cart_pole_soft_wall_dynamics_pybullet import cart_pole_dynamics
from utils import *

project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
# 添加lib目录
lib_path = os.path.join(project_root, 'lib')
if lib_path not in sys.path:
    sys.path.append(lib_path)

import DE_cuda_solver

def load_or_generate_wall_motion(load_wall_motion, total_sim_steps):
    """Load or generate wall motion data"""
    if load_wall_motion:
        wall_motion = scipy.io.loadmat('Hz_contact_experiment/Hz_contact_noise/wall_motion_100s.mat')
        print(colored("Loading wall_motion from file", 'red'))
        return wall_motion, [], []
    else:
        delta_d_left_rand = 0.0
        delta_d_right_rand = 0.0
        list_delta_d_left = []
        list_delta_d_right = []
        
        for i in range(total_sim_steps):
            delta_d_left_rand += np.random.normal(0.0, 0.2)*delta_T_dyn
            delta_d_right_rand += np.random.normal(0.0, 0.2)*delta_T_dyn
            delta_d_left = 0.03*np.sin(10*np.pi*i/1000) + delta_d_left_rand
            delta_d_right = 0.03*np.sin(10*np.pi*i/1000) + delta_d_right_rand
            
            list_delta_d_left.append(delta_d_left)
            list_delta_d_right.append(delta_d_right)
            
        return None, list_delta_d_left, list_delta_d_right

def run_simulation(env, solver, wall_motion, list_delta_d_left, list_delta_d_right, load_wall_motion, total_sim_steps):
    """Run the main simulation loop"""
    
    # Initialize tracking variables
    time_all = 0.0
    num_iter_all = 0
    ct_planned_contact = 0
    
    # Initialize trajectory storage
    x_traj = []
    u_traj = []
    contact_force_traj = []
    time_traj = []
    f_obj_traj = []
    list_num_iter = []
    list_time_spend_all = []
    num_opt_cuts_traj = []
    num_feas_cuts_traj = []
    enable_warmstart = False
    
    u_input = 0.0
    
    for i_loop in range(total_sim_steps):
        # print(f"Run time: {i_loop}")
        
        # Get wall motion
        if load_wall_motion:
            delta_d_left = wall_motion['delta_d_left'][0][i_loop]
            delta_d_right = wall_motion['delta_d_right'][0][i_loop]
        else:
            delta_d_left = list_delta_d_left[i_loop]
            delta_d_right = list_delta_d_right[i_loop]
        
        # Start logging on first iteration
        if i_loop == 0:
            env.start_logging()
            
        # Forward propagate dynamics
        state = env.forward(u=u_input, deltaT=delta_T_dyn, delta_d_left=delta_d_left, delta_d_right=delta_d_right)
        
        # Stop logging on last iteration
        if i_loop == total_sim_steps-1:
            env.stop_logging()
        
        # Extract state variables
        x1, dx1 = state['x'], state['dx']
        theta1, dtheta1 = state['theta'], state['dtheta']
        contact_force = state['contact_force']
        # print("=====================")
        # print("pos:", x1)
        # print("theta:", theta1)
        # print("speed:", dx1)
        # print("dtheta:", dtheta1)
        
        # Store trajectories
        x_traj.append(np.array([x1, theta1, dx1, dtheta1]))
        contact_force_traj.append(contact_force)
        
        # Update initial state for GBD
        current_state = np.array([x1, dx1, theta1, dtheta1])
        left_wall_pos = d_left + delta_d_left
        right_wall_pos = d_right + delta_d_right
        wall_pos = np.array([right_wall_pos, left_wall_pos])
        # print("left_wall_pos:",left_wall_pos)
        # print("right_wall_pos:", right_wall_pos)

        if(i_loop != 0):
            enable_warmstart = True
            
        solver.UpdateCartPoleSystem(current_state, wall_pos)
        solve_start = time.time()
        solution = solver.Solve(enable_warmstart)
        solver_time = time.time() - solve_start
        # print(str(solver_time*1000) + "ms")
        # print(colored(f"Speed {1/solver_time:.2f} Hz", 'green'))

        # break
        # print("solution",solution["param"])
        # 获取适应度值
        fitness = solution["fitness"]
        # print(fitness)
        # 获取参数数组
        parameters = solution["param"]
        # print(parameters)
        u_input = parameters[0]

        u_traj.append(u_input)
        f_obj_traj.append(fitness)
        print("fitness:",fitness,"u:",u_input)

        # break
        # Update constraints
        # h_theta = compute_h_theta(left_wall_pos, right_wall_pos)
        
    #     # Solve GBD
    #     t1 = time.time()
    #     sol = solver.main_loop(x0_GBD, h_theta)
    #     solve_time = time.time() - t1
        
    #     print(colored(f"Speed {1/(solve_time)} Hz", 'green'))
        
    #     # Update control input
    #     u_input = sol['control']
    #     u_traj.append([u_input])
        
    #     # Update statistics if contact is planned
    #     if sol['planned_contact']:
    #         update_statistics(sol, solve_time, i_loop, time_all, num_iter_all,
    #                         ct_planned_contact, time_traj, f_obj_traj,
    #                         list_num_iter, list_time_spend_all,
    #                         num_opt_cuts_traj, num_feas_cuts_traj)
            
    #         ct_planned_contact += 1
    #         time_all += solve_time
    #         num_iter_all += sol['num_iter']
    
    # return (x_traj, u_traj, contact_force_traj, time_traj, f_obj_traj,
    #         list_num_iter, list_time_spend_all, num_opt_cuts_traj,
    #         num_feas_cuts_traj)

def compute_h_theta(c_left, c_right):
    """Compute h_theta constraints"""
    return np.array([[0.0], [0.0], 
                    [-c_right+d_max], [c_right], 
                    [-c_left+d_max], [c_left],
                    [x_ub[0]], [-x_lb[0]], 
                    [x_ub[1]], [-x_lb[1]], 
                    [x_ub[2]], [-x_lb[2]], 
                    [x_ub[3]], [-x_lb[3]], 
                    [u_max], [u_max],
                    [lam_max], [0.0],
                    [lam_max], [0.0]])

def update_statistics(sol, solve_time, i_loop, time_all, num_iter_all,
                     ct_planned_contact, time_traj, f_obj_traj,
                     list_num_iter, list_time_spend_all,
                     num_opt_cuts_traj, num_feas_cuts_traj):
    """Update simulation statistics"""
    list_num_iter.append(sol['num_iter'])
    list_time_spend_all.append(solve_time)
    f_obj_traj.append(sol['cost'])
    time_traj.append((i_loop+1)*delta_T_dyn)
    num_opt_cuts_traj.append(sol['num_opt_cut'])
    num_feas_cuts_traj.append(sol['num_feas_cut'])
    
    print(colored(f"MPC Spending on average {1000*time_all/(ct_planned_contact+1)} ms, "
                 f"or {(ct_planned_contact+1)/time_all} Hz", 'green'))
    print(colored(f"The number of iterations are {list_num_iter}", 'green'))
    print(colored(f"The average number of iterations is "
                 f"{1.0*num_iter_all/(ct_planned_contact+1)}", 'green'))

def save_results(x_traj, time_traj, f_obj_traj, list_time_spend_all,
                num_opt_cuts_traj, num_feas_cuts_traj, list_num_iter,
                use_Gurobi=False):
    """Save simulation results"""
    if use_Gurobi:
        scipy.io.savemat('saved_results/t_spend_Gurobi_1.mat',
                        mdict={'time_traj': np.array(time_traj),
                               'cost_Gurobi': np.array(f_obj_traj),
                               'time_Gurobi': np.array(list_time_spend_all),
                               'num_iter_traj': np.array(list_num_iter),
                               'x_traj': np.array(x_traj),
                               'N': N,
                               'dT': dT})
    else:
        scipy.io.savemat('saved_results/t_spend_Benders_1.mat',
                        mdict={'time_traj': np.array(time_traj),
                               'cost_Benders': np.array(f_obj_traj),
                               'time_Benders': np.array(list_time_spend_all),
                               'num_iter_traj': np.array(list_num_iter),
                               'opt_cuts_traj': np.array(num_opt_cuts_traj),
                               'feas_cuts_traj': np.array(num_feas_cuts_traj),
                               'x_traj': np.array(x_traj),
                               'N': N,
                               'dT': dT})

def main():
    # Configuration
    load_wall_motion = True
    use_Gurobi = False

    solver = DE_cuda_solver.Create()
    print("Solver object created successfully")

    solver.init_solver(0)
    print("Solver initialized successfully")
    
    # Initialize system
    """Initialize the cart-pole system and simulation parameters"""
    env = cart_pole_dynamics(mc, mp, ll, k1, k2, d_left, d_right, d_max, 
                                u_max, x_ini, theta_ini, dx_ini, dtheta_ini, 1)

    total_sim_steps = int(T_sim/delta_T_dyn)
    # total_sim_steps = 10
    
    # Load or generate wall motion
    wall_motion, list_delta_d_left, list_delta_d_right = load_or_generate_wall_motion(
        load_wall_motion, total_sim_steps)
    
    # # Run simulation
    results = run_simulation(env, solver, wall_motion, list_delta_d_left,
                           list_delta_d_right, load_wall_motion, total_sim_steps)
    
    # Unpack results
    # print(results)

if __name__ == '__main__':
    main()