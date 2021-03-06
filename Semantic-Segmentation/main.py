"""
choose Udacity-carnd-advanced-deep-learning AMI
choose P2 instance with GPU Compute instance type
You are good to GO.

"""
import os.path
import tensorflow as tf
import helper
import warnings
from distutils.version import LooseVersion
import project_tests as tests

from tensorflow.python.platform import gfile
from tensorflow.core.protobuf import saved_model_pb2
from tensorflow.python.util import compat

# Check TensorFlow Version
assert LooseVersion(tf.__version__) >= LooseVersion('1.0'), 'Please use TensorFlow version 1.0 or newer.  You are using {}'.format(tf.__version__)
print('TensorFlow Version: {}'.format(tf.__version__))

# Check for a GPU

if not tf.test.gpu_device_name():
    warnings.warn('No GPU found. Please use a GPU to train your neural network.')
else:
    print('Default GPU Device: {}'.format(tf.test.gpu_device_name()))


def write_graph(model_filename):
    with tf.Session() as sess:
        with gfile.FastGFile(model_filename, 'rb') as f:
            data = compat.as_bytes(f.read())
            sm = saved_model_pb2.SavedModel()
            sm.ParseFromString(data)
            g_in = tf.import_graph_def(sm.meta_graphs[0].graph_def)

    LOGDIR='.'
    train_writer = tf.summary.FileWriter(LOGDIR)
    train_writer.add_graph(sess.graph)

"""
# tensorboard --logdir=.
model_filename ='./data/vgg/saved_model.pb'
write_graph(model_filename)
"""

def load_vgg(sess, vgg_path):
    """
    Load Pretrained VGG Model into TensorFlow.
    :param sess: TensorFlow Session
    :param vgg_path: Path to vgg folder, containing "variables/" and "saved_model.pb"
    :return: Tuple of Tensors from VGG model (image_input, keep_prob, layer3_out, layer4_out, layer7_out)
    """
    # TODO: Implement function
    #   Use tf.saved_model.loader.load to load the model and weights
    vgg_tag = 'vgg16'
    vgg_input_tensor_name = 'image_input:0'
    vgg_keep_prob_tensor_name = 'keep_prob:0'
    vgg_layer3_out_tensor_name = 'layer3_out:0'
    vgg_layer4_out_tensor_name = 'layer4_out:0'
    vgg_layer7_out_tensor_name = 'layer7_out:0'
    
    tf.saved_model.loader.load(sess, [vgg_tag], vgg_path)
    graph = tf.get_default_graph()
    image_input = graph.get_tensor_by_name(vgg_input_tensor_name)
    keep_prob = graph.get_tensor_by_name(vgg_keep_prob_tensor_name)
    layer3_out = graph.get_tensor_by_name(vgg_layer3_out_tensor_name)
    layer4_out = graph.get_tensor_by_name(vgg_layer4_out_tensor_name)
    layer7_out = graph.get_tensor_by_name(vgg_layer7_out_tensor_name)

    return image_input, keep_prob, layer3_out, layer4_out, layer7_out
 
tests.test_load_vgg(load_vgg, tf)


def layers(vgg_layer3_out, vgg_layer4_out, vgg_layer7_out, num_classes):
    """
    Create the layers for a fully convolutional network.  Build skip-layers using the vgg layers.
    :param vgg_layer3_out: TF Tensor for VGG Layer 3 output
    :param vgg_layer4_out: TF Tensor for VGG Layer 4 output
    :param vgg_layer7_out: TF Tensor for VGG Layer 7 output
    :param num_classes: Number of classes to classify
    :return: The Tensor for the last layer of output
    """
    # TODO: Implement function

    """
    # Adopted from Q&A
    conv_1x1 = tf.layers.conv2d(vgg_layer3_out, num_classes, 1, padding='same', 
                                kernel_regularizer=tf.contrib.layers.l2_regularizer(1e-3))
    output = tf.layers.conv2d_transpose(conv_1x1, num_classes, 4, 2, padding='same',
                                kernel_regularizer=tf.contrib.layers.l2_regularizer(1e-3))

    # Adopted from Lesson 10: Scene Understanding "8. FNCN-8 - Decoder"
    input = tf.add(input, pool_4)
    input = tf.layers.conv2d_transpose(input, num_classes, 4, strides=(2, 2))
    input = tf.add(input, pool_3)
    Input = tf.layers.conv2d_transpose(input, num_classes, 16, strides=(8, 8))
    output = tf.layers.conv2d_transpose(input, num_classes, 4, strides=(2, 2))

    # Sample from the first review
    conv_1x1_7 = tf.layers.conv2d(vgg_layer7_out, num_classes, 1, padding='same', 
        kernel_regularizer=tf.contrib.layers.l2_regularizer(reg_param),
        kernel_initializer=tf.truncated_normal_initializer(stddev=0.01))
    """

    # 
    reg_param = 1e-3

    # Adopted from "Python Deep Learning Cookbook"
    # vgg_layer7_out => decode_layer1_preskip0
    decode_layer1_preskip0 = tf.layers.conv2d_transpose(vgg_layer7_out, 512, (2, 2), (2, 2), name='decode_layer1_preskip0',
        kernel_regularizer=tf.contrib.layers.l2_regularizer(reg_param),
        kernel_initializer=tf.truncated_normal_initializer(stddev=0.01))
    # vgg_layer4_out => decode_layer1_preskip1
    decode_layer1_preskip1 = tf.layers.conv2d(vgg_layer4_out, 512, (1, 1), (1, 1), name='decode_layer1_preskip1',
        kernel_regularizer=tf.contrib.layers.l2_regularizer(reg_param),
        kernel_initializer=tf.truncated_normal_initializer(stddev=0.01))
    # decode_layer1_preskip0 + decode_layer1_preskip1 => decode_layer1_out
    decode_layer1_out = tf.add(decode_layer1_preskip0, decode_layer1_preskip1, name='decode_layer1_out')

    # decode_layer1_out => decode_layer2_preskip0
    decode_layer2_preskip0 = tf.layers.conv2d_transpose(decode_layer1_out, 256, (2, 2), (2, 2), name='decode_layer2_preskip0',
        kernel_regularizer=tf.contrib.layers.l2_regularizer(reg_param),
        kernel_initializer=tf.truncated_normal_initializer(stddev=0.01))
    # vgg_layer3_out => decode_layer2_out
    decode_layer2_preskip1 = tf.layers.conv2d(vgg_layer3_out, 256, (1, 1), (1, 1), name='decode_layer2_preskip1',
        kernel_regularizer=tf.contrib.layers.l2_regularizer(reg_param),
        kernel_initializer=tf.truncated_normal_initializer(stddev=0.01))
    # decode_layer2_preskip0 + decode_layer2_preskip1 => decode_layer2_out
    decode_layer2_out = tf.add(decode_layer2_preskip0, decode_layer2_preskip1, name='decode_layer2_out')

    decode_layer3_out = tf.layers.conv2d_transpose(decode_layer2_out, 128, (2, 2), (2, 2), name='decode_layer3_out',
        kernel_regularizer=tf.contrib.layers.l2_regularizer(reg_param),
        kernel_initializer=tf.truncated_normal_initializer(stddev=0.01))
    decode_layer4_out = tf.layers.conv2d_transpose(decode_layer3_out, 64, (2, 2), (2, 2), name='decode_layer4_out',
        kernel_regularizer=tf.contrib.layers.l2_regularizer(reg_param),
        kernel_initializer=tf.truncated_normal_initializer(stddev=0.01))
    decode_layer5_out = tf.layers.conv2d_transpose(decode_layer4_out, num_classes, (2, 2), (2, 2), name='fcn_out',
        kernel_regularizer=tf.contrib.layers.l2_regularizer(reg_param),
        kernel_initializer=tf.truncated_normal_initializer(stddev=0.01))
    return decode_layer5_out

tests.test_layers(layers)


def optimize(nn_last_layer, correct_label, learning_rate, num_classes):
    """
    Build the TensorFLow loss and optimizer operations.
    :param nn_last_layer: TF Tensor of the last layer in the neural network
    :param correct_label: TF Placeholder for the correct label image
    :param learning_rate: TF Placeholder for the learning rate
    :param num_classes: Number of classes to classify
    :return: Tuple of (logits, train_op, cross_entropy_loss)
    """
    # TODO: Implement function
    logits = tf.reshape(nn_last_layer, (-1, num_classes), name="logits")
    labels = tf.reshape(correct_label, (-1, num_classes), name="labels")


    #cross_entropy_loss = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(logits=logits, labels=labels))
    cross_entropy_loss = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits_v2(logits=logits, labels=labels))
    #train_op = tf.train.AdamOptimizer(learning_rate).minimize(cross_entropy_loss)

    # Adapted from the suggestion in the first review
    l2_loss = sum(tf.get_collection(tf.GraphKeys.REGULARIZATION_LOSSES))
    loss = tf.reduce_mean(cross_entropy_loss + l2_loss)
    train_op = tf.train.AdamOptimizer(learning_rate).minimize(loss)

    return (logits, train_op, cross_entropy_loss)

tests.test_optimize(optimize)


def train_nn(sess, epochs, batch_size, get_batches_fn, train_op, cross_entropy_loss, input_image,
             correct_label, keep_prob, learning_rate):
    """
    Train neural network and print out the loss during training.
    :param sess: TF Session
    :param epochs: Number of epochs
    :param batch_size: Batch size
    :param get_batches_fn: Function to get batches of training data.  Call using get_batches_fn(batch_size)
    :param train_op: TF Operation to train the neural network
    :param cross_entropy_loss: TF Tensor for the amount of loss
    :param input_image: TF Placeholder for input images
    :param correct_label: TF Placeholder for label images
    :param keep_prob: TF Placeholder for dropout keep probability
    :param learning_rate: TF Placeholder for learning rate
    """

    # TODO: Implement function
    for epoch in range(epochs):
        epoch_loss = 0
        epoch_size = 0
        print("Start epoch [{}]".format(epoch))
        for image, label in get_batches_fn(batch_size):
            print("Shape - Image[{}] Label[{}]".format(image.shape, label.shape))
            # Training
            _, loss = sess.run([train_op, cross_entropy_loss],
            feed_dict={input_image:image, correct_label:label,keep_prob:0.5,learning_rate:1e-4})
            print("Loss at batch: {}".format(loss))
            #print("len(image) at batch: {}".format(len(image)))
            epoch_loss += loss * len(image)
            epoch_size += len(image)
            #print("epoch_loss at batch: {}".format(epoch_loss))
            #print("epoch_size at batch: {}".format(epoch_size))
        #print("Loss at epoch {}: {}".format(epoch, epoch_loss/epoch_size))
        print("Loss at epoch {}: {}".format(epoch, epoch_loss))

tests.test_train_nn(train_nn)

def run():
    num_classes = 2
    image_shape = (160, 576)
    data_dir = './data'
    runs_dir = './runs'
    tests.test_for_kitti_dataset(data_dir)

    # Download pretrained vgg model
    helper.maybe_download_pretrained_vgg(data_dir)

    # OPTIONAL: Train and Inference on the cityscapes dataset instead of the Kitti dataset.
    # You'll need a GPU with at least 10 teraFLOPS to train on.
    #  https://www.cityscapes-dataset.com/

    with tf.Session() as sess:
        # Path to vgg model
        vgg_path = os.path.join(data_dir, 'vgg')
        # Create function to get batches
        get_batches_fn = helper.gen_batch_function(os.path.join(data_dir, 'data_road/training'), image_shape)

        # OPTIONAL: Augment Images for better results
        #  https://datascience.stackexchange.com/questions/5224/how-to-prepare-augment-images-for-neural-network

        # TODO: Build NN using load_vgg, layers, and optimize function
        temp = set(tf.global_variables())

        input_image, keep_prob, layer3_out, layer4_out, layer7_out = load_vgg(sess, vgg_path)
        layer_output = layers(layer3_out, layer4_out, layer7_out, num_classes)
        nn_last_layer = layer_output

        correct_label = tf.placeholder("float", shape=(None, 160, 576, num_classes))
        learning_rate = tf.placeholder(tf.float32, name="learning_rate")
 
        logits, train_op, cross_entropy_loss = optimize(nn_last_layer, correct_label, learning_rate, num_classes)

        # TODO: Train NN using the train_nn function
        epochs = 23
        # Changed the batch size suggested in the first review as follows:
        # Batch size used is on larger side. Try setting a value around 2 or 4.
        # larger batch sizes might speed up the training but can degrade the quality of the model at the same time.
        #batch_size = 16
        batch_size = 4

        sess.run(tf.variables_initializer(set(tf.global_variables()) - temp))

        train_nn(sess, epochs, batch_size, get_batches_fn, train_op, cross_entropy_loss, input_image,
             correct_label, keep_prob, learning_rate)
        # TODO: Save inference data using helper.save_inference_samples
        #  helper.save_inference_samples(runs_dir, data_dir, sess, image_shape, logits, keep_prob, input_image)
        helper.save_inference_samples(runs_dir, data_dir, sess, image_shape, logits, keep_prob, input_image)

        # OPTIONAL: Apply the trained model to a video


if __name__ == '__main__':
    run()