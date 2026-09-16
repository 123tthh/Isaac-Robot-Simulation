import json
import pandas as pd
import matplotlib.pyplot as plt

# 1. Load the JSON data
with open('safe_pose_mass.json', 'r') as f:
    data = json.load(f)

# 2. Extract summary and trial data
summary_df = pd.DataFrame(data['summary'])
trials_df = pd.DataFrame(data['trials'])

# 3. Calculate average drift and duration per mass from trials
avg_metrics = trials_df.groupby('Mass_kg').agg({
    'max_abs_yaw_drift_deg': 'mean',
    'duration': 'mean'
}).reset_index()

# Merge data for plotting
merged_df = pd.merge(summary_df, avg_metrics, on='Mass_kg')
safe_range = data.get('safe_mass_range_kg', None)

# 4. Set up the plot
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

# --- Subplot 1: Success Rate ---
ax1.plot(merged_df['Mass_kg'], merged_df['rate'], marker='o', color='#1f77b4', 
         linewidth=2, markersize=8, label='Success Rate')
ax1.set_ylabel('Success Rate', fontsize=12)
ax1.set_title('Effect of Pallet Mass on Conveyor Simulation Success and Dynamics', 
              fontsize=14, pad=15)
ax1.set_ylim(-0.1, 1.2)
ax1.grid(True, linestyle='--', alpha=0.6)

if safe_range:
    # Highlight the safe range interval
    ax1.axvspan(safe_range[0], safe_range[1], color='#2ca02c', alpha=0.2, 
                label=f'Safe Mass Range: [{safe_range[0]}, {safe_range[1]}] kg')
    ax1.legend(loc='lower right', fontsize=11)

# --- Subplot 2: Dynamics (Yaw Drift & Duration) ---
color1 = '#ff7f0e'
ax2.plot(merged_df['Mass_kg'], merged_df['max_abs_yaw_drift_deg'], marker='s', 
         color=color1, linewidth=2, markersize=7, label='Avg Max Yaw Drift (deg)')
ax2.set_xlabel('Mass (kg)', fontsize=12)
ax2.set_ylabel('Max Yaw Drift (deg)', color=color1, fontsize=12)
ax2.tick_params(axis='y', labelcolor=color1)
ax2.grid(True, linestyle='--', alpha=0.6)
ax2.set_ylim(0, 8)

# Twin axis for Duration (since they use different units)
ax3 = ax2.twinx()
color2 = '#9467bd'
ax3.plot(merged_df['Mass_kg'], merged_df['duration'], marker='^', 
         color=color2, linewidth=2, markersize=7, label='Avg Duration (s)')
ax3.set_ylabel('Duration (s)', color=color2, fontsize=12)
ax3.tick_params(axis='y', labelcolor=color2)
ax3.set_ylim(0.5, 2.0)

# Combine legends for subplot 2
lines_1, labels_1 = ax2.get_legend_handles_labels()
lines_2, labels_2 = ax3.get_legend_handles_labels()
ax2.legend(lines_1 + lines_2, labels_1 + labels_2, loc='upper center', 
           bbox_to_anchor=(0.5, -0.15), ncol=2, fontsize=11)

# Render and Save
plt.tight_layout()
plt.savefig('mass_effect.png', dpi=300, bbox_inches='tight')