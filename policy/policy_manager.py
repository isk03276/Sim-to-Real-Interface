import tensorflow as tf
from agent import PPOAgent
from policy.policy import *
import os
import environments

class PolicyManager:
    def __init__(self, policy_path):
        self.load_policy(policy_path)

    def load_policy(self, file_path):
        tf.reset_default_graph()
        with tf.Session() as session:
            with tf.variable_scope(MASTER_NAME) as scope:
                policy = get_policy(env_opts, session)
                master_agent = PPOAgent(policy, session, 'master-0', env_opts)
            saver = tf.train.Saver(max_to_keep=1)
            saver.restore(session, tf.train.latest_checkpoint(file_path))


    def get_action(self, ob):
        pass