# Locus ML service — named constants.
# All magic numbers used in model.py, train.py, and main.py are defined here.

# ===== MODEL ARCHITECTURE =====
SEQ_LEN = 20          # Number of timesteps fed to the LSTM per aircraft track
NUM_FEATURES = 8      # Feature vector size: lat, lon, alt_m, vel_lat, vel_lon, vel_alt, gdop, divergence_m
HIDDEN_SIZE = 128     # LSTM hidden state dimension
NUM_LSTM_LAYERS = 2   # Stacked LSTM depth
DROPOUT_RATE = 0.3    # LSTM dropout for regularisation
NUM_CLASSES = 2       # Output classes: 0 = normal, 1 = anomalous

# ===== CLASS LABELS =====
NORMAL_LABEL = 0           # Label assigned to non-spoofed tracks
ANOMALY_LABEL = 1          # Label assigned to spoofed / anomalous tracks
ANOMALY_CLASS_INDEX = 1    # Softmax output index for the anomalous class

# ===== TRAINING HYPERPARAMETERS =====
TRAIN_SPLIT = 0.8              # Fraction of data used for training vs validation
TRAIN_BATCH_SIZE = 32          # Mini-batch size during training
VAL_BATCH_SIZE = 64            # Mini-batch size during validation pass
LEARNING_RATE = 1e-3           # Adam optimiser learning rate
WEIGHT_DECAY = 1e-4            # Adam L2 regularisation (weight decay)
GRADIENT_CLIP = 1.0            # Max gradient norm for clipping
DEFAULT_EPOCHS = 20            # Default training epoch count from CLI
DEFAULT_RETRAIN_EPOCHS = 20    # Epoch count for online retraining triggered by new labels
NORMAL_CLASS_WEIGHT = 1.0      # Cross-entropy loss weight for normal class
ANOMALY_CLASS_WEIGHT = 4.0     # Cross-entropy loss weight for anomalous class (4× penalty for missed spoofs)
STD_EPSILON = 1e-8             # Small epsilon added to std-dev to avoid division by zero

# ===== SYNTHETIC DATASET SIZES =====
N_NORMAL = 800           # Number of normal (non-spoofed) synthetic tracks per training run
N_ANOMALOUS = 200        # Number of anomalous (spoofed) synthetic tracks per training run
NUM_ANOMALY_TYPES = 4    # Anomaly types: position jump, velocity flip, altitude teleport, position freeze

# ===== SYNTHETIC DATA GENERATION RANGES =====
# Geographic bounds (UK airspace region)
LAT_MIN = 48.0    # Minimum latitude for synthetic track generation (degrees)
LAT_MAX = 54.0    # Maximum latitude for synthetic track generation (degrees)
LON_MIN = -2.0    # Minimum longitude for synthetic track generation (degrees)
LON_MAX = 12.0    # Maximum longitude for synthetic track generation (degrees)

# Altitude range
ALT_MIN = 5000.0    # Minimum altitude for synthetic tracks (feet)
ALT_MAX = 12000.0   # Maximum altitude for synthetic tracks (feet)

# Velocity ranges (degrees/step or m/s for vertical)
VEL_LAT_MIN = 0.001    # Min lateral velocity component (degrees/step)
VEL_LAT_MAX = 0.005    # Max lateral velocity component (degrees/step)
VEL_LON_MIN = 0.002    # Min longitudinal velocity component (degrees/step)
VEL_LON_MAX = 0.008    # Max longitudinal velocity component (degrees/step)
VEL_ALT_MIN = -2.0     # Min vertical velocity (m/s; negative = descending)
VEL_ALT_MAX = 2.0      # Max vertical velocity (m/s; positive = climbing)

# Normal track quality ranges
GDOP_MIN = 1.0       # Minimum GDOP for normal synthetic tracks
GDOP_MAX = 5.0       # Maximum GDOP for normal synthetic tracks
DIVERGENCE_MIN = 0.0    # Minimum MLAT–ADS-B divergence for normal tracks (metres)
DIVERGENCE_MAX = 50.0   # Maximum MLAT–ADS-B divergence for normal tracks (metres)

# Track step scaling
TRACK_STEP_SCALE = 0.1    # Scaling factor applied to velocity when advancing each timestep

# ===== ANOMALY GENERATION PARAMETERS =====
# Shared injection window (anomalies are injected somewhere in the middle of the sequence)
ANOMALY_WINDOW_MIN = 5     # Earliest step index at which an anomaly is injected
ANOMALY_WINDOW_MAX = 15    # Latest step index at which an anomaly is injected

# Position-jump spoofing (sudden lat/lon teleport)
POSITION_JUMP_MIN = 0.02    # Minimum position jump magnitude (degrees)
POSITION_JUMP_MAX = 0.08    # Maximum position jump magnitude (degrees)

# Anomalous quality indicators for position-jump tracks
ANOMALY_GDOP_MIN = 8.0        # Minimum GDOP on anomalous tracks (elevated = bad geometry)
ANOMALY_GDOP_MAX = 20.0       # Maximum GDOP on anomalous tracks
ANOMALY_DIVERGENCE_MIN = 500.0     # Minimum MLAT–ADS-B divergence on anomalous tracks (metres)
ANOMALY_DIVERGENCE_MAX = 5000.0    # Maximum MLAT–ADS-B divergence on anomalous tracks (metres)

# Velocity-flip spoofing (sudden velocity reversal)
VELOCITY_FLIP_MULTIPLIER = -1.0    # Multiplier applied to velocity components to simulate reversal

# Altitude-teleport spoofing (sudden altitude jump)
ALT_JUMP_MIN = 2000.0    # Minimum altitude teleport magnitude (metres)
ALT_JUMP_MAX = 5000.0    # Maximum altitude teleport magnitude (metres)
