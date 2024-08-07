from __future__ import annotations

import matplotlib.patches as patches
import matplotlib.pyplot as plt
import pandas as pd

def with_identical_rods(
    ranges: dict[str, float],
    data_folderpath: str,
    data_filename: str,
    fig_folderpath: str,
    fig_filename: str,
    boundary_folderpath: str,
    boundary_filename: str,
) -> None:
    data_filepath: str = data_folderpath + data_filename
    df: pd.DataFrame = pd.read_table(
        data_filepath, index_col=0, header=None, delim_whitespace=True
    )
    df.head()
    df.loc[:, [2, 3]].head()

    boundary_filepath: str = boundary_folderpath + boundary_filename
    boundary: pd.DataFrame = pd.read_table(
        boundary_filepath, header=None, delim_whitespace=True
    )
    boundary.head()

    ax = plt.axes()

    ax.plot(boundary[0], boundary[1])

    # fc = face color, ec = edge color
    for index, row in df.iterrows():
        c: patches.Circle = patches.Circle(
            xy=(row[2], row[3]), radius=0.5, fc="g", ec="g"
        )
        ax.add_patch(c)

    # plt.axis('scaled')
    # set_(x|y)lim does not work when axis is scaled
    ax.set_xlim(xmin=ranges["xmin"], xmax=ranges["xmax"])
    ax.set_ylim(ymin=ranges["ymin"], ymax=ranges["ymax"])

    ax.set_aspect("equal")

    fig_filepath: str = fig_folderpath + fig_filename
    plt.savefig(fig_filepath)

    plt.close()

def with_specific_rods(
    ranges: dict[str, float],
    data_folderpath: str,
    data_filename: str,
    fig_folderpath: str,
    fig_filename: str,
    boundary_folderpath: str,
    boundary_filename: str,
) -> None:
    data_filepath: str = data_folderpath + data_filename
    df: pd.DataFrame = pd.read_table(
        data_filepath, index_col=0, header=None, delim_whitespace=True
    )
    df.head()
    df.loc[:, [2, 3]].head()

    boundary_filepath: str = boundary_folderpath + boundary_filename
    boundary: pd.DataFrame = pd.read_table(
        boundary_filepath, header=None, delim_whitespace=True
    )
    boundary.head()

    ax = plt.axes()

    ax.plot(boundary[0], boundary[1])

    specific_rods: list[int] = [0, 1, 2]

    # fc = face color, ec = edge color
    for index, row in df.iterrows():
        c: patches.Circle = patches.Circle(
            xy=(row[2], row[3]), radius=0.5, fc="g", ec="g"
        )
        if index in specific_rods:
            c = patches.Circle(xy=(row[2], row[3]), radius=0.5, fc="r", ec="r")
        ax.add_patch(c)

    # plt.axis('scaled')
    # set_(x|y)lim does not work when axis is scaled
    ax.set_xlim(xmin=ranges["xmin"], xmax=ranges["xmax"])
    ax.set_ylim(ymin=ranges["ymin"], ymax=ranges["ymax"])

    ax.set_aspect("equal")

    fig_filepath: str = fig_folderpath + fig_filename
    plt.savefig(fig_filepath)

    plt.close()

def with_head(
    ranges: dict[str, float],
    data_folderpath: str,
    data_filename: str,
    fig_folderpath: str,
    fig_filename: str,
    boundary_folderpath: str | None = None,
    boundary_filename_list: list[str] | None = None,
) -> None:
    data_filepath: str = data_folderpath + data_filename
    df: pd.DataFrame = pd.read_table(
        data_filepath, index_col=0, header=None, delim_whitespace=True
    )
    df.head()
    df.loc[:, [2, 3]].head()

    ax = plt.axes()

    if boundary_folderpath is not None and boundary_filename_list is not None:
        for boundary_filename in boundary_filename_list:
            boundary_filepath: str = boundary_folderpath + boundary_filename
            boundary: pd.DataFrame = pd.read_table(
                boundary_filepath, header=None, delim_whitespace=True
            )
            ax.plot(boundary[0], boundary[1], color="b")

    # Calculate offsets for periodic boundary conditions
    width = ranges['xmax'] - ranges['xmin']
    height = ranges['ymax'] - ranges['ymin']
    offsets = [(-width, 0), (width, 0), (0, -height), (0, height), (-width, -height), (-width, height), (width, -height), (width, height)]

    # fc = face color, ec = edge color
    for index, row in df.iterrows():
        rod_type = row.iloc[-1]
        x, y = row[2], row[3]

        # Default color
        fc_h, ec_h = "grey", "grey"
        fc_b, ec_b = "grey", "grey"
        if rod_type == 0:
            fc_h, ec_h = "r", "r"
            fc_b, ec_b = "g", "g"
        elif rod_type == 1:
            fc_h, ec_h = "black", "black"
            fc_b, ec_b = "blue", "blue"

        for dx, dy in [(0, 0)] + offsets:
            fc = fc_h if row[1] == 0 else fc_b
            ec = ec_h if row[1] == 0 else ec_b
            c = patches.Circle(
                (x + dx, y + dy), radius=0.5, fc=fc, ec=ec
            )
            ax.add_patch(c)
            if fc == "black":  # Add ID text to black circles
                ax.text(x + dx, y + dy, str(index), color="white", ha='center', va='center', fontsize=8)

    # plt.axis("scaled")
    # set_(x|y)lim does not work when axis is scaled
    ax.set_xlim(xmin=ranges["xmin"], xmax=ranges["xmax"])
    ax.set_ylim(ymin=ranges["ymin"], ymax=ranges["ymax"])

    ax.set_aspect("equal")

    fig_filepath: str = fig_folderpath + fig_filename
    plt.savefig(fig_filepath)

    plt.close()
