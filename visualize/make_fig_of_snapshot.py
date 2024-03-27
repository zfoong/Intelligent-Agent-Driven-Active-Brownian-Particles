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

    # fc = face color, ec = edge color
    for index, row in df.iterrows():
        rod_type = row.iloc[-1]
        # default color is not matching type
        fc_h, ec_h = "grey", "grey"
        fc_s, ec_s = "grey", "grey"
        if rod_type == 0:
            fc_h, ec_h = "g", "g"
            fc_s, ec_s = "r", "r"
        elif rod_type == 1:
            fc_h, ec_h = "blue", "blue"
            fc_s, ec_s = "black", "black"

        c: patches.Circle = patches.Circle(
            xy=(row[2], row[3]), radius=0.5, fc=fc_h, ec=ec_h
        )
        if row[1] == 0:
            c = patches.Circle(xy=(row[2], row[3]), radius=0.5, fc=fc_s, ec=ec_s)
        ax.add_patch(c)

    # plt.axis("scaled")
    # set_(x|y)lim does not work when axis is scaled
    ax.set_xlim(xmin=ranges["xmin"], xmax=ranges["xmax"])
    ax.set_ylim(ymin=ranges["ymin"], ymax=ranges["ymax"])

    ax.set_aspect("equal")

    fig_filepath: str = fig_folderpath + fig_filename
    plt.savefig(fig_filepath)

    plt.close()
