import argparse
import os
import yaml

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input_yaml", default="")
    parser.add_argument("--output_yaml", default="")
    return parser.parse_args()


def convert(example, parent_dir):
    new_filename = os.path.join(parent_dir, example["filename"])
    example["filename"] = new_filename
    return example

def main(args):    
    with open(args.input_yaml) as f:
        examples = yaml.load(f, Loader=yaml.FullLoader)

    parent_dir = "/".join(args.input_yaml.split('/')[:-1])


    out = [convert(example, parent_dir) for example in examples]

    with open(args.output_yaml, 'w') as outfile:
        yaml.dump(out, outfile, default_flow_style=False)


if __name__ == "__main__":
    main(parse_args())