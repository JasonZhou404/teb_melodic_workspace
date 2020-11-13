# /bin/python
"""
Steps:
1. download data into the data folder
2. configure tasks and metrics in main()
3. run "python3 plotting.py"
"""

import glob
import json
import numpy as np
import os

from matplotlib.backends.backend_pdf import PdfPages
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.style
import matplotlib as mpl

matplotlib.use('Agg')
# mpl.style.use('seaborn-pastel')
mpl.style.use('seaborn-paper')


def underscore_to_capital(underscore_str):
    return ' '.join([word.capitalize() for word in underscore_str.split('_')])


def parse_txt(path):
    data = []
    with open(path, "r") as file:
        line = file.readline()
        while line:
            data.append(float(line))
            line = file.readline()
        file.close()
    return data


def plot(data, metrics):

    plt.figure(figsize=(10, 4))

    x_label = f'traversal_distance'
    y_label = f'curvature'

    plt.xlabel(f'{underscore_to_capital(x_label)}' + ' ($meter$)', fontsize='18')
    plt.ylabel(f'{underscore_to_capital(y_label)}' + ' ($meter^{-1}$)', fontsize='18')

    x_keys = [key for key in data.keys() if x_label in key]

    # First, plot the upper bound of the curvature constrants
    plt.fill_between([0, 27], [0.3, 0.3], [0.2, 0.2],
                     alpha=0.6, color='orange',
                     label='Exceed-constraint zones')

    # Second, plot the lower bound of the curvature constrants
    plt.fill_between([0, 27], [-0.2, -0.2], [-0.3, -0.3],
                     alpha=0.6, color='orange')

    # Third, plot the curvatures from different planners
    line_color = ['darkgreen', 'purple', 'red', 'blue']
    line_count = 0
    for x_key in x_keys:
        x = data[x_key]
        y_key = x_key.replace(x_label, y_label, 1)
        y = data[y_key]
        if (len(x) != len(y)):
            print(f'x length: {len(x)}; y length: {len(y)}')

        # sort (x, y) based on x
        points = sorted(zip(x, y), key=lambda point: point[0])
        x, y = zip(*points)

        plt.plot(x, y,
                 label=y_key.split('_')[0],
                 marker='.', markersize=0.5,
                 linestyle='-', linewidth=3, alpha=1.0,
                 color=line_color[line_count]
                 )
        line_count += 1


    plt.xlim((0, 27))
    plt.ylim((-0.3, 0.3))

    plt.legend(loc=2, bbox_to_anchor=(0.01, 0.98), borderaxespad=0,
               fontsize='x-large')
    handles, labels = plt.gca().get_legend_handles_labels()
    include1 = [0,1,2,3]
    include2 = [4]
    legend1 = plt.legend([handles[i] for i in include1],[labels[i] for i in include1],
              loc=1, bbox_to_anchor=(0.99, 0.80), borderaxespad=0, fontsize='x-large',
              title='Path Curvatures', title_fontsize='x-large')
    legend2 = plt.legend([handles[i] for i in include2],[labels[i] for i in include2],
              loc=4, bbox_to_anchor=(0.99, 0.02), borderaxespad=0, fontsize='x-large')
    plt.gca().add_artist(legend1)
    plt.show()


    plt.tick_params(axis='both', which='major', labelsize=14, pad=12)
    # plt.title(underscore_to_capital(metric), fontsize='x-large')
    plt.tight_layout()


def main():
    prefix = f'Open_Space_Planner'
    cases = ['fig5']
    for case in cases:
        # output path
        input = f'plots/{case}/data'
        dir_path = os.path.dirname(input)
        base_path = os.path.basename(input)

        data_list = glob.glob(f'{input}/*.txt')

        print(data_list)
        data_list.sort(reverse=False)
        metrics = {
            # key: metric name, which will be shown in the legend
            # value: path to the txt file
            "H-OBCA_curvature": data_list[2],
            "H-OBCA_traversal_distance": data_list[3],
            "TEB_curvature": data_list[6],
            "TEB_traversal_distance": data_list[7],
            "CES_curvature": data_list[0],
            "CES_traversal_distance": data_list[1],
            "DL-IAPS_curvature": data_list[4],
            "DL-IAPS_traversal_distance": data_list[5],
        }

        # list of metrics to plot. Will plot one metric per graph.
        tasks = [
            'path_curvature',
        ]

        # plot
        data = {name: parse_txt(txt_path) for name, txt_path in metrics.items()}
        for key in data.keys():
            print(f'key: {key}; data size: {len(data[key])}')
        for task in tasks:
            plot(data, metrics)
            output = os.path.join(dir_path, f'{prefix}_{task}_{case}.png')
            plt.savefig(f'{output}')
            plt.close()
        print(f'finish plotting task: {case}')
        # with PdfPages(output) as pdf:
        #     for metric in metrics:
        #         plot(data, metric)
        #         pdf.savefig()
        #         plt.close()


if __name__ == "__main__":
    main()
