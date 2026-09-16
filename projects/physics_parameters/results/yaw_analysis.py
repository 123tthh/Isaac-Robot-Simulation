import json
import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd

# 1. Read JSON data
with open('safe_pose_yaw.json', 'r', encoding='utf-8') as f:
    data = json.load(f)

# Convert to DataFrames
df_trials = pd.DataFrame(data['trials'])
df_summary = pd.DataFrame(data['summary'])

# 2. Setup Plot Style
sns.set_theme(style="whitegrid")
fig, axes = plt.subplots(1, 2, figsize=(16, 6))

threshold = data['config'].get('YAW_LOCK_THRESHOLD', 3.0)
n_trials = data['config'].get('N_TRIALS', 3)

# ==========================================
# Subplot 1: Target Yaw vs Max Yaw Drift (Scatter)
# ==========================================
sns.scatterplot(
    data=df_trials,
    x='Yaw_deg',
    y='max_abs_yaw_drift_deg',
    hue='result',
    style='result',
    palette={'success': '#2ca02c', 'rail_lock': '#d62728', 'stuck': '#ff7f0e'},
    markers={'success': 'o', 'rail_lock': 'X', 'stuck': 's'},
    s=100,
    alpha=0.7,
    ax=axes[0]
)

# Draw the threshold line
axes[0].axhline(y=threshold, color='red', linestyle='--', label=f"Threshold ({threshold}°)")
axes[0].set_title('Max Yaw Drift vs. Initial Yaw Angle')
axes[0].set_xlabel('Initial Target Yaw (°)')
axes[0].set_ylabel('Max Absolute Yaw Drift (°)')
axes[0].legend(title='Result')

# ==========================================
# Subplot 2: Outcomes vs Target Yaw (Stacked Bar)
# ==========================================
axes[1].bar(df_summary['Yaw_deg'], df_summary['n_success'], 
            color='#2ca02c', label='Success', alpha=0.8, width=0.8)
axes[1].bar(df_summary['Yaw_deg'], df_summary['n_rail_lock'], 
            bottom=df_summary['n_success'], 
            color='#d62728', label='Rail Lock', alpha=0.8, width=0.8)
axes[1].bar(df_summary['Yaw_deg'], df_summary['n_stuck'], 
            bottom=df_summary['n_success'] + df_summary['n_rail_lock'], 
            color='#ff7f0e', label='Stuck', alpha=0.8, width=0.8)

axes[1].set_title('Experiment Outcomes across Initial Yaw Angles')
axes[1].set_xlabel('Initial Target Yaw (°)')
axes[1].set_ylabel('Number of Trials')
axes[1].set_yticks(range(0, n_trials + 1))
axes[1].legend(title='Outcome')

# Adjust layout and save/show
plt.tight_layout()
plt.savefig('yaw_analysis_step2_english.png', dpi=300, bbox_inches='tight')
plt.show()