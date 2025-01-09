import osimpipeline as osp
import tasks
import os

def generate_main_tasks(trial):
    # 添加逆运动学任务
    # inverse kinematics
    ik_setup_task = trial.add_task(osp.TaskIKSetup)
    trial.add_task(osp.TaskIK, ik_setup_task)
    trial.add_task(osp.TaskIKPost, ik_setup_task,
        error_markers=trial.study.error_markers)

    # 添加逆动力学任务
    # inverse dynamics
    id_setup_task = trial.add_task(osp.TaskIDSetup, ik_setup_task)
    trial.add_task(osp.TaskID, id_setup_task)
    # trial.add_task(osp.TaskIDPost, id_setup_task)

    return ik_setup_task, id_setup_task

def generate_unperturbed_tasks(study, subject, trial, 
        initial_time, final_time):
    # 添加初始猜测任务
    # Initial guess creation
    # ----------------------
    guess_fpath = ''
    # 根据下面的参数来组合设置初始猜测任务
    # 参数为：缩放比例、网格间隔、保留强度、隐式多体、隐式肌腱、周期性标志、创建和插入标志、收敛容差、约束容差
    # ???还有一些意义不是很清楚的，需要逐渐了解
    scales             = [0.1, 0.5, 1.0]
    mesh_intervals     = [0.04, 0.03, 0.02]
    reserves           = [200, 20, 0]
    implicit_multibody = [True, True, False]
    implicit_tendons   = [True, True, False]
    periodic_flags     = [False, True, True]
    create_and_insert  = [False, False, True]
    convergence_tols   = [1e-1, 1e-2, 1e-2]
    constraint_tols    = [1e-2, 1e-2, 1e-3]  
    # 将参数组合在一起
    zipped = zip(scales, mesh_intervals, reserves,
                 implicit_multibody, implicit_tendons, 
                 periodic_flags, create_and_insert,
                 convergence_tols, constraint_tols)
    # 遍历参数组合，添加初始猜测任务
    for scale, mesh, reserve, imp_multi, imp_ten, periodic, candi, conv_tol, const_tol in zipped:
        trial.add_task(
            tasks.TaskMocoUnperturbedWalkingGuess,
            initial_time, final_time, 
            mesh_interval=mesh, 
            walking_speed=study.walking_speed,
            periodic=periodic,
            cost_scale=scale,
            reserve_strength=reserve,
            implicit_multibody_dynamics=imp_multi,
            implicit_tendon_dynamics=imp_ten,
            guess_fpath=guess_fpath,
            create_and_insert_guess=candi,
            convergence_tolerance=conv_tol,
            constraint_tolerance=const_tol)
        # 根据参数组合设置初始猜测任务的名字，以便后续寻找结果文件
        guess_name = f'unperturbed_guess_mesh{mesh}_scale{scale}_reserve{reserve}'
        if periodic: guess_name += '_periodic'
        guess_fpath = os.path.join(
            study.config['results_path'], 'guess', subject.name, 
            f'{guess_name}.sto')

    # 正常行走预测任务
    # Unperturbed walking
    # -------------------
    # 如果启用了初始猜测，则添加正常行走预测任务
    if study.config['unperturbed_initial_guess']:
        # 设置初始猜测文件路径
        guess_fpath = os.path.join(
            study.config['results_path'],
            'unperturbed', subject.name, 
            'unperturbed.sto')

    trial.add_task(
        tasks.TaskMocoUnperturbedWalking,
        initial_time, final_time, 
        mesh_interval=0.01, 
        walking_speed=study.walking_speed,
        guess_fpath=guess_fpath,
        periodic=True,
        create_and_insert_guess=False)

    # 不同腰椎刚度的正常行走预测任务
    # Unperturbed walking w/ different lumbar stiffnesses
    # ---------------------------------------------------
    # 如果subject是subject01，则添加不同腰椎刚度的正常行走预测任务
    # if subject.name == 'subject01':
    #     for lumbar_stiffness in study.lumbar_stiffnesses:
    #         if lumbar_stiffness == 1.0: continue
    #         trial.add_task(
    #             tasks.TaskMocoUnperturbedWalking,
    #             initial_time, final_time, 
    #             mesh_interval=0.01, 
    #             walking_speed=study.walking_speed,
    #             guess_fpath=unperturbed_guess_fpath,
    #             periodic=True,
    #             lumbar_stiffness=lumbar_stiffness)

def generate_perturbed_tasks(study, subject, trial, 
        initial_time, final_time, right_strikes, 
        left_strikes):
    # 遍历时间、矢状面力矩、跖屈力矩等生成并添加模拟外骨骼辅助行走的任务，并查看是否添加画图任务
    for time in study.times:
        for torque in study.torques:
            for subtalar in study.subtalar_peak_torques:
                torque_parameters = [torque / 100.0, 
                                     time / 100.0, 
                                     study.rise / 100.0, 
                                     study.fall / 100.0]
                subtalar_peak_torque = subtalar / 100.0
                # for lumbar_stiffness in study.lumbar_stiffnesses:
                #     if (not lumbar_stiffness == 1.0) and (not subject.name == 'subject01'): continue
                #     trial.add_task(
                #         tasks.TaskMocoPerturbedWalking,
                #         initial_time, final_time, right_strikes, left_strikes,
                #         torque_parameters=torque_parameters,
                #         walking_speed=study.walking_speed,
                #         side='right',
                #         subtalar_torque_perturbation=bool(subtalar),
                #         subtalar_peak_torque=subtalar_peak_torque,
                #         lumbar_stiffness=lumbar_stiffness)
                #     trial.add_task(
                #         tasks.TaskMocoPerturbedWalkingPost,
                #         trial.tasks[-1])

                # 遍历是否使用坐标执行器，结合前面参数，添加模拟外骨骼辅助行走的任务      
                for coordact in [False, True]:
                    trial.add_task(
                            tasks.TaskMocoPerturbedWalking,
                            initial_time, final_time, right_strikes, left_strikes,
                            torque_parameters=torque_parameters,
                            walking_speed=study.walking_speed,
                            side='right',
                            subtalar_torque_perturbation=bool(subtalar),
                            subtalar_peak_torque=subtalar_peak_torque,
                            lumbar_stiffness=1.0,
                            use_coordinate_actuators=coordact)
                    # 如果启用了扰动画图任务，则添加扰动画图任务
                    if study.config['enable_perturbed_plotting_tasks']:
                        trial.add_task(
                                tasks.TaskMocoPerturbedWalkingPost,
                                trial.tasks[-1])

