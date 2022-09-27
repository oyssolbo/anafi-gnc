
import pathlib
import numpy as np

class GenericOutputSaver():

    def __init__(self, config, base_dir, output_category, output_type, environment):
        # TODO: Include that output_category must be e.g. ground_truth or estimates
        
        self.config = config[output_category][output_type]

        self.output_dir = f"{base_dir}/{output_category}"
        pathlib.Path(f"{self.output_dir}").mkdir(parents=True, exist_ok=True)

        self.environment = environment

        self.buffer_index = 0
        self.buffer_max_size = self.config["max_values_stored_in_buffer"]
        self.output_buffer = np.zeros((
            self.buffer_max_size,
            self.config["num_states"] + 1) # +1 because of timestamp
        )

        # Create file with an informative header
        self.output_filename = f"{base_dir}/{output_category}/{self.config['name']}.txt"
        with open(self.output_filename, "w+") as file_desc:
            file_desc.write(self.config["file_header"])

        self.topic_name = self.config["topic"][self.environment]

    def _save_output(self, output):
        # Write buffer array to file if full
        if self.buffer_index >= self.buffer_max_size:
            with open(self.output_filename, "a") as file_desc:
                np.savetxt(file_desc, self.output_buffer)
            self.buffer_index = 0

        self.output_buffer[self.buffer_index] = output
        self.buffer_index += 1