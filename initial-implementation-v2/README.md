# RL-Based Controller Simulation (Improved Implementation)

This directory contains the simulation and training code for a Reinforcement Learning (RL) based controller.

## Contents

* `rl_env_scaledObs.py` — Custom RL environment with scaled observations
* `rl_train.py` — Script to train the controller using PPO
* `runsim_scaledObs.py` — Script to run simulations with a trained model

## Getting Started

### 1️⃣ Set Up Python Virtual Environment

```bash
python3 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 2️⃣ Install Dependencies

```bash
pip install -r requirements.txt
```

### 3️⃣ Train the Model (Optional)

To train the controller from scratch:

```bash
python rl_train.py
```

📂 **Trained model checkpoints** will be saved automatically in the checkpoints folder.

### 4️⃣ Run the Simulation

Use the following command to simulate using a trained model:

```bash
python runsim_scaledObs.py
```

📌 **Make sure to update the model filename inside `runsim_scaledObs.py` to point to your saved checkpoint.**

---

## Notes

* Only **scaled observation states** are used in this implementation.
* Ensure your checkpoints are correctly placed as referenced in the simulation script.
* `runsim_scaledObs.py` is the primary script for running trained models.
* In this implementation, the number of observation states is 20.
* Continuous waypoint navigation is implemented.
* The best performing checkpoint so far is:`checkpoints_from_8_6M/ppo_model_2300000_steps.zip`

---


![sim](https://github.com/user-attachments/assets/86054425-71d7-41da-8d23-b255abac6f66)
