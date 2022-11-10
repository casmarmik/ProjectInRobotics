import sys
import os
import numpy as np

# Network stuff 
# link to installation of cuda driver https://www.tensorflow.org/install/pip
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'
import tensorflow as tf
tf.config.list_physical_devices('GPU')
print("Num GPUs Available: ", len(tf.config.list_physical_devices('GPU')))
from tensorflow.keras.models import Model
from tensorflow.keras.optimizers import Adam
from tensorflow.keras.layers import Input, Conv2D, Flatten, Dense
from tensorflow.keras.layers import MaxPooling2D
from tensorflow.keras.layers import Dropout, BatchNormalization
from tensorflow.keras.callbacks import LearningRateScheduler, ModelCheckpoint
from tensorflow.keras.preprocessing.image import ImageDataGenerator
from tensorflow.keras.utils import plot_model

class CNN():
    def build_cnn_layer(self, net, n_filters, use_batch_norm=False):
        net = Conv2D(n_filters, 3, activation='relu', padding='valid', kernel_initializer='he_normal')(net)
        if use_batch_norm:
            net = BatchNormalization()(net)
        net = MaxPooling2D((2, 2))(net)
        return net

    def build_model(self, input_size, n_layers, n_filters, use_batch_norm, plot_and_print_summary=False):
        """
        Builds the CNN model

        Parameters
        ----------
        input_size (tuple): The inputs size of the image for the model
        n_layers (int): The number of conv layers
        n_filters (itn): The number of filters in the first layer
        use_batch_norm (bool): If True there will be used batch normalization
        """
        inputs = Input(input_size)
        net = inputs

        # Create the conv blocks
        for _ in range(n_layers):
            net = self.build_cnn_layer(net, n_filters, use_batch_norm)
            print(net.get_shape())
            n_filters *= 2

        # Create dense layers
        net = Flatten()(net)
        print(net.get_shape())
        
        net = Dense(256, activation='relu')(net)
        net = Dropout(0.5)(net)
        net = BatchNormalization()(net)
        print(net.get_shape())

        net = Dense(3, activation='softmax')(net)
        self.model = Model(inputs=inputs, outputs=net)
        print(net.get_shape())


        if plot_and_print_summary:
            self.model.summary()

            # Try to make a PDF with a plot of the model
            try:
                plot_model(self.model, to_file='model.pdf', show_shapes=True)
            except:
                print("To get plot of model fix error:", sys.exc_info()[0])

    def train(self, train, val, steps_per_epoch, epochs, lr):
        """
        Trains the model

        Parameters
        ----------
        train_set (generator): A image+mask generator for the training data
        val_set (generator): A image+mask generator for the validation data
        steps_per_epoch (int): The number of steps per epoch
        epochs (int): The number of epochs
        lr (float): The learning rate
        """
        # Compile the model
        self.model.compile(optimizer=Adam(learning_rate=lr), loss='categorical_crossentropy', metrics=['accuracy'])

        # Make a learning rate scheduler
        def scheduler(epoch):
            if epoch < 3:
                return 0.0001
            else:
                return 0.00005

        lr_schedule = LearningRateScheduler(scheduler)

        # Create a ModelCheckpoint callback to save teh best version of the model as it trains
        checkpoint_name = "retina_extraction.hdf5"
        model_checkpoint = ModelCheckpoint(checkpoint_name, monitor='val_loss', verbose=1, save_best_only=True)

        # Train the model
        print("training the model")
        self.model.fit(train, steps_per_epoch=steps_per_epoch, epochs=epochs, 
                       validation_steps=10, callbacks=[model_checkpoint, lr_schedule], batch_size=10)

        # Load the best version of the model
        self.model.load_weights(checkpoint_name)

    def predict(self, test_data, test_targets):
        """
        Predict on the given data

        Parameters
        ----------
        Test data: test data
        test_targets: test targets
        """
        for img in test_data:
            prediction = self.model.predict(img)
            print("prediction and targets")
            print(prediction)


# This doesn't work locally
if __name__ == '__main__':
    # gather data
    train_dataset = ImageDataGenerator(validation_split=0.2, rescale=1/255.0)
    train_ds = train_dataset.flow_from_directory("vision/data/object_classification/cropped_images/", target_size=(280, 310), subset="training", batch_size=10)
   
    val_ds =train_dataset.flow_from_directory("vision/data/object_classification/cropped_images/", target_size=(280, 310), subset="validation", batch_size=10)

    input_size = (280, 310, 3)  # The input size for the model
    n_layers = 2  # The number of con layers in the network
    n_filters = 32  # The first layers number of filters
    use_batch_norm = True  # Use batch normalization or not

    cnn = CNN()
    cnn.build_model(input_size, n_layers, n_filters, False)

    epochs = 20  # The number of epochs to train over
    steps_per_epoch = int(len(train_ds) / 10) # The number of training batches to preform for each epoch
    lr = 1e-4  # The learning rate to start with
    cnn.train(train_ds, [], steps_per_epoch, epochs, lr)

