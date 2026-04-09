import h5py
import numpy as np

def crop_h5_by_index(input_file, output_file, start_idx, end_idx):
    with h5py.File(input_file, "r") as f_in, h5py.File(output_file, "w") as f_out:
        for key in f_in.keys():
            data = f_in[key]
            
            if len(data.shape) == 0:  # scalar
                cropped = data[()]
                print(f"{key}: scalar unchanged")
            elif start_idx < data.shape[0]:
                cropped = data[start_idx:min(end_idx, data.shape[0])]
                print(f"{key}: {data.shape} → {cropped.shape}")
            else:
                cropped = np.empty((0,) + data.shape[1:])
                print(f"{key}: empty slice")
            
            dset = f_out.create_dataset(key, data=cropped)
            for attr in data.attrs:
                dset.attrs[attr] = data.attrs[attr]

# Example usage
crop_h5_by_index(
    input_file="../past_logs/20260409_114851_LOG.h5",
    output_file="cropped.h5",
    start_idx=3888,
    end_idx=5548
)
