import os

import osimpipeline as osp
import tasks
import helpers

# 用于设置缩放
def scale_setup_fcn(util, mset, sset, ikts):

    m = util.Measurement('torso', mset)
    m.add_markerpair('RASI', 'CLAV')
    m.add_markerpair('LASI', 'CLAV')
    m.add_markerpair('LPSI', 'C7')
    m.add_markerpair('RPSI', 'C7')
    m.add_markerpair('RASI',' RACR')
    m.add_markerpair('LASI', 'LACR')
    m.add_bodyscale('torso')

    m = util.Measurement('pelvis_x', mset)
    m.add_markerpair('RASI', 'RPSI')
    m.add_markerpair('LASI', 'LPSI')
    m.add_bodyscale('pelvis', 'X')

    m = util.Measurement('pelvis_y', mset)
    m.add_markerpair('LPSI', 'LHJC')
    m.add_markerpair('RPSI', 'RHJC')
    m.add_markerpair('RASI', 'RHJC')
    m.add_markerpair('LASI', 'LHJC')
    m.add_bodyscale('pelvis', 'Y')

    m = util.Measurement('pelvis_z', mset)
    m.add_markerpair('RPSI', 'LPSI')
    m.add_markerpair('RASI', 'LASI')
    m.add_bodyscale('pelvis', 'Z')

    m = util.Measurement('thigh', mset)
    m.add_markerpair('LHJC', 'LLFC')
    m.add_markerpair('LHJC', 'LMFC')
    m.add_markerpair('RHJC', 'RMFC')
    m.add_markerpair('RHJC', 'RLFC')
    m.add_bodyscale_bilateral('femur')

    m = util.Measurement('shank', mset)
    m.add_markerpair('LLFC', 'LLMAL')
    m.add_markerpair('LMFC', 'LMMAL')
    m.add_markerpair('RLFC', 'RLMAL')
    m.add_markerpair('RMFC', 'RMMAL')
    m.add_bodyscale_bilateral('tibia')

    m = util.Measurement('foot', mset)
    m.add_markerpair('LCAL', 'LMT5')
    m.add_markerpair('LCAL', 'LTOE')
    m.add_markerpair('RCAL', 'RTOE')
    m.add_markerpair('RCAL',' RMT5')
    m.add_bodyscale_bilateral('talus')
    m.add_bodyscale_bilateral('calcn')
    m.add_bodyscale_bilateral('toes')

    m = util.Measurement('humerus', mset)
    m.add_markerpair('LSJC', 'LMEL')
    m.add_markerpair('LSJC', 'LLEL')
    m.add_markerpair('RSJC', 'RLEL')
    m.add_markerpair('RSJC', 'RMEL')
    m.add_bodyscale_bilateral('humerus')

    m = util.Measurement('radius_ulna', mset)
    m.add_markerpair('LLEL', 'LFAradius')
    m.add_markerpair('LMEL', 'LFAulna')
    m.add_markerpair('RMEL', 'RFAulna')
    m.add_markerpair('RLEL', 'RFAradius')
    m.add_bodyscale_bilateral('ulna')
    m.add_bodyscale_bilateral('radius')
    m.add_bodyscale_bilateral('hand')

    ikts.add_ikmarkertask_bilateral('ACR', True, 100.0)
    ikts.add_ikmarkertask('CLAV', True, 250.0)
    ikts.add_ikmarkertask('C7', True, 250.0)
    ikts.add_ikmarkertask_bilateral('ASH', True, 10.0)
    ikts.add_ikmarkertask_bilateral('PSH', True, 10.0)
    ikts.add_ikmarkertask_bilateral('SJC', False, 10.0)
    ikts.add_ikmarkertask_bilateral('LEL', True, 50.0)
    ikts.add_ikmarkertask_bilateral('MEL', True, 50.0)
    ikts.add_ikmarkertask_bilateral('FAsuperior', False, 0.0)
    ikts.add_ikmarkertask_bilateral('FAradius', True, 50.0)
    ikts.add_ikmarkertask_bilateral('FAulna', True, 50.0)

    ikts.add_ikmarkertask_bilateral('ASI', True, 100.0)
    ikts.add_ikmarkertask_bilateral('PSI', True, 50.0)
    ikts.add_ikmarkertask_bilateral('HJC', True, 100.0)
    ikts.add_ikmarkertask_bilateral('LFC', True, 50.0)
    ikts.add_ikmarkertask_bilateral('MFC', True, 50.0)
    ikts.add_ikmarkertask_bilateral('KJC', False, 100.0)
    ikts.add_ikmarkertask_bilateral('LMAL', True, 50.0)
    ikts.add_ikmarkertask_bilateral('MMAL', True, 50.0)
    ikts.add_ikmarkertask_bilateral('AJC', False, 100.0)
    ikts.add_ikmarkertask_bilateral('CAL', True, 25.0)
    ikts.add_ikmarkertask_bilateral('TOE', True, 25.0)
    ikts.add_ikmarkertask_bilateral('MT5', True, 25.0)

    ikts.add_ikmarkertask_bilateral('EJC', False, 0.0)
    ikts.add_ikmarkertask_bilateral('TH1', False, 0.0)
    ikts.add_ikmarkertask_bilateral('TH2', False, 0.0)
    ikts.add_ikmarkertask_bilateral('TH3', False, 0.0)
    ikts.add_ikmarkertask_bilateral('TB1', False, 0.0)
    ikts.add_ikmarkertask_bilateral('TB2', False, 0.0)
    ikts.add_ikmarkertask_bilateral('TB3', False, 0.0)
    ikts.add_ikmarkertask_bilateral('UA1', False, 0.0)
    ikts.add_ikmarkertask_bilateral('UA2', False, 0.0)
    ikts.add_ikmarkertask_bilateral('UA3', False, 0.0)

    ikts.add_ikcoordinatetask('pelvis_tilt', True, 1.0)
    ikts.add_ikcoordinatetask('pelvis_list', True, 1.0)
    ikts.add_ikcoordinatetask_bilateral('hip_flexion', True, 1.0)
    ikts.add_ikcoordinatetask_bilateral('hip_rotation', False, 1.0, manual_value=0.0)
    ikts.add_ikcoordinatetask_bilateral('ankle_angle', True, 1.0, manual_value=0.0)
    ikts.add_ikcoordinatetask('lumbar_bending', True, 0.0)
    ikts.add_ikcoordinatetask('lumbar_rotation', True, 0.0)
    ikts.add_ikcoordinatetask_bilateral('arm_flex', False, 1.0)
    ikts.add_ikcoordinatetask_bilateral('arm_rot', False, 1.0)
    ikts.add_ikcoordinatetask_bilateral('elbow_flex', False, 1.0)
    ikts.add_ikcoordinatetask_bilateral('pro_sup', False, 1.0)


def add_to_study(study):
    
    # Add subject to study
    # --------------------
    # 添加subject到study中，参数为：subject编号、体重、身高
    subject = study.add_subject(1, 72.84, 1.808)

    # 条件参数
    cond_args = dict()
    subject.cond_args = cond_args

    # 添加静态条件
    static = subject.add_condition('static')
    # 添加静态条件下的试验
    static_trial = static.add_trial(1, omit_trial_dir=True)

    # 设置缩放任务，参数为：初始时间、结束时间、运动学试验、缩放函数、额外文件依赖
    # `os.path.basename(__file__)` should be `subject01.py`.
    scale_setup_task = subject.add_task(osp.TaskScaleSetup,
            init_time=0,
            final_time=0.5, 
            mocap_trial=static_trial,
            edit_setup_function=scale_setup_fcn,
            addtl_file_dep=['dodo.py', os.path.basename(__file__)])

    subject.add_task(osp.TaskScale,
            scale_setup_task=scale_setup_task,
            ignore_unused_markers=True)

    # 根据体重和身高缩放肌肉最大等长力
    # Scale max isometric forces based on mass and height
    # ---------------------------------------------------
    # 复制模型段质量
    subject.add_task(tasks.TaskCopyModelSegmentMasses)
    # 缩放肌肉最大等长力
    subject.add_task(tasks.TaskScaleMuscleMaxIsometricForce)
    # 设置缩放模型文件路径
    subject.scaled_model_fpath = os.path.join(subject.results_exp_path,
        f'{subject.name}_final.osim')
    # 设置仿真模型文件路径，就是用缩放后的模型
    subject.sim_model_fpath = os.path.join(subject.results_exp_path,
        f'{subject.name}_final.osim')

    # 添加正常行走条件
    # walk2 condition
    # ---------------
    walk2 = subject.add_condition('walk2', metadata={'walking_speed': 1.25})

    # 添加正常行走试验
    # Trial to use
    # 数据中正常行走事件时间点
    gait_events = dict()
    gait_events['right_strikes'] = [1.18, 2.28, 3.38, 4.49] 
    gait_events['left_toeooffs'] = [1.36, 2.46, 3.56]
    gait_events['left_strikes'] = [1.73, 2.83, 3.94] 
    gait_events['right_toeoffs'] = [1.92, 3.02, 4.12]

    # 添加正常行走试验，试验编号为1，包含行走事件时间点，省略试验目录
    walk2_trial = walk2.add_trial(1,
            gait_events=gait_events,
            omit_trial_dir=True,
            )
    # 增加任务：更新地面反作用力标签
    walk2_trial.add_task(tasks.TaskUpdateGroundReactionLabels)
    # 增加任务：过滤地面反作用力
    walk2_trial.add_task(tasks.TaskFilterGroundReactions)
    # 增加任务：生成地面反作用力步态标记
    walk2_trial.add_task(osp.TaskGRFGaitLandmarks, min_time=0.5, max_time=5.0)

    # 生成主任务：生成逆运动学和逆动力学任务
    # Inverse kinematics and inverse dynamics
    ik_setup_task, id_setup_task = helpers.generate_main_tasks(walk2_trial)

    # 设置选定实验室数据的初始时间、结束时间、步态事件时间点
    initial_time = 3.38
    final_time = 4.49
    duration = 1.11
    # 右脚事件时间点
    right_strikes = [3.38, 4.49]
    # 左脚事件时间点
    left_strikes = [3.94]
    # 增加任务：计算关节角度标准差
    walk2_trial.add_task(
        tasks.TaskComputeJointAngleStandardDeviations, 
        ik_setup_task)
    # 增加任务：修剪跟踪数据
    walk2_trial.add_task(
        tasks.TaskTrimTrackingData, 
        ik_setup_task, id_setup_task, 
        initial_time, final_time)

    # 增加任务：生成未扰动正常行走任务
    # unperturbed walking tasks
    # -------------------------
    helpers.generate_unperturbed_tasks(study, subject, walk2_trial, 
        initial_time, final_time)

    # 增加任务：生成外骨骼辅助行走任务
    # perturbed walking tasks
    # -----------------------
    helpers.generate_perturbed_tasks(study, subject, walk2_trial, 
        initial_time, final_time, right_strikes, left_strikes)
