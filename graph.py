import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# 1. Carregar dados
try:
    df = pd.read_csv('dado.txt')
    df = df.dropna()
except FileNotFoundError:
    print("Erro: Arquivo 'dado.txt' não encontrado.")
    exit()

# 2. Configuração de fonte (SEM LaTeX, mas parecida)
plt.rcParams.update({
    'font.size': 10,
    'font.family': 'serif',
    'font.serif': ['Computer Modern Roman', 'DejaVu Serif'],  # parecido com Overleaf
    'axes.labelsize': 11,
    'xtick.labelsize': 9,
    'ytick.labelsize': 9,
    'legend.fontsize': 9,
    'figure.dpi': 300
})

fig, ax1 = plt.subplots(figsize=(7, 4))

# Cores
color_v = '#1f77b4'
color_j = '#d62728'

# --- VELOCIDADE ---
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Velocity (m/s)', color=color_v, fontweight='bold')

ax1.plot(df['time'], df['speed_filt'],
         color=color_v,
         linewidth=2.2,
         alpha=0.7,  # mais transparente
         label='Velocity')

ax1.tick_params(axis='y', labelcolor=color_v)
ax1.grid(True, linestyle='--', alpha=0.2)

# --- DETECÇÃO DE FASE ---
change_points = df.index[df['test_phase'] != df['test_phase'].shift()].tolist()

jerk_filtered = np.full(len(df), np.nan)

if len(change_points) > 1:
    idx_transition = change_points[1]
    t_transition = df.loc[idx_transition, 'time']

    # Jerk só no Stage 2
    jerk_filtered[idx_transition:] = df['jerk'][idx_transition:]

    # Background das fases
    ax1.axvspan(df['time'].min(), t_transition,
                color='#1f77b4', alpha=0.08)

    ax1.axvspan(t_transition, df['time'].max(),
                color='#2ca02c', alpha=0.08)

    # Linha divisória
    ax1.axvline(x=t_transition,
                color='black', linestyle='--', linewidth=1)

# --- JERK ---
ax2 = ax1.twinx()
ax2.set_ylabel('Jerk ($m/s^3$)', color=color_j, fontweight='bold')

ax2.plot(df['time'], jerk_filtered,
         color=color_j,
         linewidth=1.8,
         linestyle='-',
         alpha=0.9,
         label='Jerk')

ax2.tick_params(axis='y', labelcolor=color_j)

# --- TEXTOS DAS FASES ---
if len(change_points) > 1:
    xmin = df['time'].min()
    xmax = df['time'].max()

    x_norm_transition = (t_transition - xmin) / (xmax - xmin)

    ax1.text(x_norm_transition / 2, 1.05,
             'Stage 1: S&K',
             transform=ax1.transAxes,
             ha='center',
             fontsize=10,
             fontweight='bold')

    ax1.text((1 + x_norm_transition) / 2, 1.05,
             'Stage 2: SIMC',
             transform=ax1.transAxes,
             ha='center',
             fontsize=10,
             fontweight='bold')

# --- LIMPEZA ---
ax1.spines['top'].set_visible(False)
ax2.spines['top'].set_visible(False)

# --- LEGENDA (menor) ---
lines_1, labels_1 = ax1.get_legend_handles_labels()
lines_2, labels_2 = ax2.get_legend_handles_labels()

ax1.legend(lines_1 + lines_2,
           labels_1 + labels_2,
           loc='upper left',
           frameon=False,
           fontsize=8)

# Ajuste layout
fig.tight_layout(rect=[0, 0, 1, 0.95])

# Salvar
plt.savefig('figure_etfa_final.pdf', bbox_inches='tight')
plt.savefig('figure_etfa_final.png', bbox_inches='tight', dpi=300)

print("Finalizado!")