import json
import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd

# 1. Load the data
with open('safe_pose_pitch.json', 'r', encoding='utf-8') as f:
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
# Subplot 1: Target Pitch vs Max Yaw Drift (Scatter)
# ==========================================
sns.scatterplot(
    data=df_trials,
    x='Pitch_deg',
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
axes[0].set_title('Max Yaw Drift vs. Initial Pitch Angle')
axes[0].set_xlabel('Initial Target Pitch (°)')
axes[0].set_ylabel('Max Absolute Yaw Drift (°)')

# Force legend to show all classes even if some are missing (like rail_lock)
handles, labels = axes[0].get_legend_handles_labels()
axes[0].legend(handles=handles, labels=labels, title='Result')

# ==========================================
# Subplot 2: Outcomes vs Target Pitch (Stacked Bar + Line)
# ==========================================
# Bar chart for outcomes
axes[1].bar(df_summary['Pitch_deg'], df_summary['n_success'], 
            color='#2ca02c', label='Success', alpha=0.8, width=0.8)
axes[1].bar(df_summary['Pitch_deg'], df_summary['n_rail_lock'], 
            bottom=df_summary['n_success'], 
            color='#d62728', label='Rail Lock', alpha=0.8, width=0.8)
axes[1].bar(df_summary['Pitch_deg'], df_summary['n_stuck'], 
            bottom=df_summary['n_success'] + df_summary['n_rail_lock'], 
            color='#ff7f0e', label='Stuck', alpha=0.8, width=0.8)

axes[1].set_title('Experiment Outcomes and Dynamic Z-Offset')
axes[1].set_xlabel('Initial Target Pitch (°)')
axes[1].set_ylabel('Number of Trials')
axes[1].set_yticks(range(0, n_trials + 1))
axes[1].legend(title='Outcome', loc='upper left')

# Superimpose the Dynamic Z Offset as a line plot on a secondary Y-axis
ax2 = axes[1].twinx()
ax2.plot(df_summary['Pitch_deg'], df_summary['Z_offset_mm'], 
         color='blue', marker='d', linestyle=':', linewidth=2.5, markersize=7, 
         label='Dynamic Z Offset (mm)')
ax2.set_ylabel('Compensated Z Offset (mm)', color='blue', fontweight='bold')
ax2.tick_params(axis='y', labelcolor='blue')

# Add legend for the secondary axis
lines, labels = ax2.get_legend_handles_labels()
ax2.legend(lines, labels, loc='center right')

# Adjust layout and show/save
plt.tight_layout()
plt.savefig('pitch_analysis_step3_english.png', dpi=300, bbox_inches='tight')
plt.show()