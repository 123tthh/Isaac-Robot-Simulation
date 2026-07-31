import json
import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd

# Read data
with open('safe_pose_y.json', 'r', encoding='utf-8') as f:
    data = json.load(f)

# Extract data into pandas DataFrames
df_trials = pd.DataFrame(data['trials'])
df_summary = pd.DataFrame(data['summary'])

# Setup plot style
sns.set_theme(style="whitegrid")
fig, axes = plt.subplots(1, 2, figsize=(16, 6))

# Get threshold from config
threshold = data['config'].get('YAW_LOCK_THRESHOLD', 2.0)
n_trials = data['config'].get('N_TRIALS', 3)

# ==========================================
# Subplot 1: Y Offset vs Max Yaw Drift (Scatter)
# ==========================================
sns.scatterplot(
    data=df_trials,
    x='Y_mm',
    y='max_abs_yaw_drift_deg',
    hue='result',
    style='result',
    palette={'success': '#2ca02c', 'rail_lock': '#d62728', 'stuck': '#ff7f0e'},
    markers={'success': 'o', 'rail_lock': 'X', 'stuck': 's'},
    s=100,
    alpha=0.7,
    ax=axes[0]
)
axes[0].axhline(y=threshold, color='red', linestyle='--', label=f"Threshold ({threshold}°)")
axes[0].set_title('Max Yaw Drift across Y Offsets')
axes[0].set_xlabel('Y Offset (mm)')
axes[0].set_ylabel('Max Absolute Yaw Drift (deg)')
axes[0].legend(title='Result')

# ==========================================
# Subplot 2: Outcomes (Counts) vs Y Offset (Stacked Bar)
# ==========================================
# Use the 'summary' data to plot exact outcome counts per Y offset
axes[1].bar(df_summary['Y_mm'], df_summary['n_success'], 
            color='#2ca02c', label='Success', alpha=0.8, width=0.8)
axes[1].bar(df_summary['Y_mm'], df_summary['n_rail_lock'], 
            bottom=df_summary['n_success'], 
            color='#d62728', label='Rail Lock', alpha=0.8, width=0.8)
axes[1].bar(df_summary['Y_mm'], df_summary['n_stuck'], 
            bottom=df_summary['n_success'] + df_summary['n_rail_lock'], 
            color='#ff7f0e', label='Stuck', alpha=0.8, width=0.8)

axes[1].set_title('Experiment Outcomes across Y Offsets')
axes[1].set_xlabel('Y Offset (mm)')
axes[1].set_ylabel('Number of Trials')
axes[1].set_yticks(range(0, n_trials + 1))  # Y-axis ticks from 0 to N_TRIALS
axes[1].legend(title='Outcome')

# Adjust layout and save
plt.tight_layout()
plt.savefig('yaw_analysis_english.png', dpi=300, bbox_inches='tight')
plt.show()