import os
import opensim as osim

# 主程序入口
if __name__ == "__main__":
    import argparse

    # 获取当前文件的绝对路径
    root_dir = os.path.abspath(os.path.dirname(os.path.abspath(__file__)))

    # 创建参数解析器
    parser = argparse.ArgumentParser()
    # 添加试验名称参数
    parser.add_argument('--trial', type=str)

    # 添加未扰动标志参数
    parser.add_argument('--unperturbed', dest='unperturb', action='store_true')
    # 添加踝关节扰动标志参数
    parser.add_argument('--ankle-perturbation', dest='ankle_perturb', action='store_true')
    # 添加峰值扭矩参数
    parser.add_argument('--peak-torque', dest='peak_torque', type=int)
    # 添加峰值时间参数
    parser.add_argument('--peak-time', dest='peak_time', type=int)
    # 添加延迟参数
    parser.add_argument('--delay', dest='delay', type=int)
    # 设置默认参数值
    parser.set_defaults(unperturb=False, ankle_perturb=False, 
                        peak_torque=None, peak_time=None, delay=None)

    # 解析命令行参数
    args = parser.parse_args()

    # 根据是否有踝关节扰动选择子目录
    subdir = 'ankle_perturb' if args.ankle_perturb else 'moco'
    # 构建结果目录路径
    dir = os.path.join(root_dir, 'results', 'experiments', 'subject01',
        args.trial, subdir)
    # 如果提供了峰值扭矩、峰值时间和延迟，则更新目录路径
    if args.peak_torque and args.peak_time and args.delay:
        dir = os.path.join(dir, 
            f'torque{args.peak_torque}_time{args.peak_time}_delay{args.delay}')

    # 加载模型
    model = osim.Model(os.path.join(dir, 'model.osim'))
    # 初始化模型系统
    model.initSystem()

    # 设置解决方案文件名
    solution_fname = 'perturb.sto'
    # 如果选择未扰动，则更新文件名
    solution_fname = 'unperturbed.sto' if args.unperturb else solution_fname
    # 如果提供了峰值扭矩、峰值时间和延迟，则更新解决方案文件名
    if args.peak_torque and args.peak_time and args.delay:
        solution_fname = f'perturb_torque{args.peak_torque}_time{args.peak_time}_delay{args.delay}.sto'
    # 加载解决方案轨迹
    solution = osim.MocoTrajectory(os.path.join(dir, solution_fname))
    # 显示模型运动
    osim.VisualizerUtilities.showMotion(model, solution.exportToStatesTable())