# RL-Based Controller Simulation(initial implementation)

This directory contains code to run simulations using Reinforcement Learning (RL) trained controllers. There are two versions of the simulation:

- `runsim_scaledObs.py`: Uses scaled observation states.
- `runsim_vecN.py`: Uses vector-normalized observations.

## Steps to Run the Simulation

1. **Create a Python Virtual Environment**  
   Run the following in your terminal:

   ```bash
   python3 -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

2. **Install Required Packages**  
   Make sure you're in the root directory, then run:

   ```bash
   pip install -r requirements.txt
   ```

3. **Run the Simulation**

   - To run the simulation with **scaled observations**:

     ```bash
     python runsim_scaledObs.py
     ```

     📌 **Note**: Inside `runsim_scaledObs.py`, update the model filename to point to your correct trained model.

   - To run the simulation with **vector-normalized observations**:

     ```bash
     python runsim_vecN.py
     ```

     📌 **Note**: Inside `runsim_vecN.py`, also update the model filename accordingly.

## Additional Notes

- Make sure the trained model files are available in the correct path as referenced in the script.
- Only observation states are scaled in `runsim_scaledObs.py`.
- Use `runsim_vecN.py` only if the model was trained with vector normalization applied to observations.

