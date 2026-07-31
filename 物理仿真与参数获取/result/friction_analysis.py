import json
import pandas as pd
import matplotlib.pyplot as plt

# 1. 加载 JSON 数据
with open('safe_pose.json', 'r') as f:
    data = json.load(f)

# 2. 提取统计摘要与具体试验数据
summary_df = pd.DataFrame(data['summary'])
trials_df = pd.DataFrame(data['trials'])

# 3. 计算每个摩擦系数下试验的平均偏航角漂移与耗时
avg_metrics = trials_df.groupby('mu_roller_s').agg({
    'max_abs_yaw_drift_deg': 'mean',
    'duration': 'mean',
    'final_v': 'mean'
}).reset_index()

# 合并绘图数据
merged_df = pd.merge(summary_df, avg_metrics, on='mu_roller_s')
safe_range = data.get('safe_mu_s_range', None)

# 4. 配置画布
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

# --- 子图 1: 成功率与可行域 ---
ax1.plot(merged_df['mu_roller_s'], merged_df['rate'], marker='o', color='#1f77b4', 
         linewidth=2, markersize=8, label='Success Rate')
ax1.set_ylabel('Success Rate', fontsize=12)
ax1.set_title('Effect of Roller Friction ($\mu_s$) on Simulation Success and Dynamics', 
              fontsize=14, pad=15)
ax1.set_ylim(-0.1, 1.2)
ax1.grid(True, linestyle='--', alpha=0.6)

if safe_range:
    # 使用绿色高亮标注出安全/可行域空间
    ax1.axvspan(safe_range[0], safe_range[1], color='#2ca02c', alpha=0.2, 
                label=f'Feasible Region: [{safe_range[0]}, {safe_range[1]}]')
    ax1.legend(loc='lower right', fontsize=11)

# --- 子图 2: 系统动态 (偏航与耗时) ---
color1 = '#ff7f0e'
ax2.plot(merged_df['mu_roller_s'], merged_df['max_abs_yaw_drift_deg'], marker='s', 
         color=color1, linewidth=2, markersize=7, label='Avg Max Yaw Drift (deg)')
ax2.set_xlabel('Roller Static Friction ($\mu_s$)', fontsize=12)
ax2.set_ylabel('Max Yaw Drift (deg)', color=color1, fontsize=12)
ax2.tick_params(axis='y', labelcolor=color1)
ax2.grid(True, linestyle='--', alpha=0.6)
ax2.set_ylim(0, 8)

# 耗时的第二 Y 轴
ax3 = ax2.twinx()
color2 = '#9467bd'
ax3.plot(merged_df['mu_roller_s'], merged_df['duration'], marker='^', 
         color=color2, linewidth=2, markersize=7, label='Avg Duration (s)')
ax3.set_ylabel('Duration (s)', color=color2, fontsize=12)
ax3.tick_params(axis='y', labelcolor=color2)
ax3.set_ylim(0.5, 2.0)

# 合并子图 2 的图例
lines_1, labels_1 = ax2.get_legend_handles_labels()
lines_2, labels_2 = ax3.get_legend_handles_labels()
ax2.legend(lines_1 + lines_2, labels_1 + labels_2, loc='upper center', 
           bbox_to_anchor=(0.5, -0.15), ncol=2, fontsize=11)

plt.tight_layout()
plt.savefig('friction_effect.png', dpi=300, bbox_inches='tight')