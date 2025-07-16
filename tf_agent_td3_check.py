
import datetime
import numpy as np
import tensorflow as tf
from tf_agents.environments import tf_py_environment
from tf_agents.environments import TimeLimit
from tf_agents.agents.ddpg.actor_network import ActorNetwork
from tf_agents.agents.ddpg.critic_network import CriticNetwork
from tf_agents.agents import Td3Agent


np.random.seed(1234)
tf.random.set_seed(12345)



save_path = 'C:/Users/aless/Downloads/Uni/Advanced_Deep_Learning_Models_and_Methods/Project/python_code/training_data/' + datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

# Data collection
replay_buffer_capacity = 1000000
initial_collect_steps = 100 # total number of steps collected with a random policy. Every time the steps TimeLimit is reached, the environment is reset

# Agent
fc_layer_params = (128, 128,)

# Training
train_env_steps_limit = 200 # maximum number of steps in the TimeLimit of the training environment
collect_steps_per_iteration = 200 # maximum number of steps in each episode

epochs = 3000
batch_size = 512
learning_rate = 3e-4
checkpoint_dir = save_path + '/ckpts'
policy_dir = save_path + '/policies'
ckpts_interval = 10 # every how many epochs to store a checkpoint during training

# Evaluation
eval_env_steps_limit = 400 # maximum number of steps in the TimeLimit of the evaluation environment
num_eval_episodes = 5
eval_interval = 50 # interval for evaluation and policy saving, =epochs for evaluation only at the end




global_step = tf.compat.v1.train.get_or_create_global_step() # global counter of the steps

actor_net = ActorNetwork((19,), (4,), fc_layer_params=fc_layer_params, activation_fn=tf.keras.activations.tanh)
critic_net = CriticNetwork(((19,), (4,)), joint_fc_layer_params=fc_layer_params, activation_fn=tf.keras.activations.relu)

agent = Td3Agent( 
                  (4,),
                  actor_network=actor_net,
                  critic_network=critic_net,
                  actor_optimizer=tf.keras.optimizers.Adam(learning_rate=learning_rate),
                  critic_optimizer=tf.keras.optimizers.Adam(learning_rate=learning_rate),
                  target_update_tau=1.0,
                  target_update_period=2,
                  gamma=0.99,
                  train_step_counter=global_step)

agent.initialize()

print("\nActor network summary and details")
print(actor_net.summary())
for i, layer in enumerate (actor_net.layers):
    print (i, layer)
    try: print ("    ",layer.activation)
    except AttributeError: print('   no activation attribute')

print("\nCritic network summary and details")
print(critic_net.summary())
for i, layer in enumerate (critic_net.layers):
    print (i, layer)
    try: print ("    ",layer.activation)
    except AttributeError: print('   no activation attribute')