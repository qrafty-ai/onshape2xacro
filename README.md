# onshape2xacro

A streamlined exporter for converting Onshape assemblies into high-quality Xacro/URDF robot descriptions.

## Why yet another exporter?

Existing exporters often output flat URDFs or require numerous API calls that can quickly drain out the annual limit. `onshape2xacro` is designed with a different philosophy:

- **Easier to change values in Xacro**: Outputs clean Xacro files instead of flat URDFs. This makes it trivial to tweak joint limits, colors, or physics parameters directly in the code without re-exporting. Also, the generated xacro macro makes it easier to integrate the robot into the project and reduce post-processing need.
- **Saves API calls via Local Processing**: Instead of downloading each part individually via the API, it fetches the entire assembly as a STEP file once. All mesh extraction and inertia calculations are then performed locally using the STEP data, resulting in a much faster and more reliable pipeline.
- **Auto-link Merging**: Automatically identifies and merges parts that are fixed together in Onshape. This produces a clean, simplified kinematic tree that matches how the robot is actually controlled, rather than having a link for every single screw.
- **Modular Export**: Optionally mirrors Onshape's subassembly hierarchy, creating reusable Xacro macros and organized file structures for complex robots.

## Installation

This project uses `uv` for modern, fast Python package management.

Use uvx directly:

```bash
uvx --from git+https://github.com/qrafty-ai/onshape2xacro onshape2xacro --help
```

or manually clone the repo:

```bash
git clone https://github.com/qrafty-ai/onshape2xacro.git
cd onshape2xacro
uv sync # or pip install -e .
```

## Usage

The export workflow is broken down into modular steps to give you full control over the process:

0. **Prerequisite**: Before exporting, rename all mates you wish to convert into URDF joints using the prefix joint_. This allows the tool to distinguish between your robot's kinematic joints and other assembly constraints.

1. **Auth**: Setup your Onshape API credentials.

    ```bash
    onshape2xacro auth login
    ```

    Follow the prompts to enter your Access and Secret keys. These are stored securely in your system keyring. You can check the status with `onshape2xacro auth status`.

2. **Fetch CAD**: Download the assembly structure and STEP assets.

    ```bash
    onshape2xacro fetch-cad <assembly_url> <local_dir>
    ```

    This generates:
    - `cad.pickle`: Cached assembly metadata.
    - `assembly.step`: The full 3D geometry of the robot.
    - `mate_values.json`: A mapping of joint IDs to their current values.

    Note that due to the limitation of onshape API (see [Limitation](#limitation)) there's no stable enough way to retrieve the current mate values of the assembly automatically. Therefore, the default generated `mate_values.json` are all 0. The preferred way is to make sure you put all mates to 0 before fetching data (you can create a [Name Position](https://cad.onshape.com/help/Content/named-positions.htm) to make this easier). If there's mate that can' be set to `0`, you can modify `mate_values.json` manually to the correct values.

3. **Modify Mate Value** (Optional):
    If you want to export the robot in a specific pose (e.g., a "zero" configuration that differs from the CAD model), edit `<local_dir>/mate_values.json` and adjust the joint angles or translations.

4. **Export**: Generate the final Xacro description.

    ```bash
    onshape2xacro export <local_dir> --output <final_xacro_dir> --format xacro
    ```

    This processes the local assets, calculates inertias based on geometry and material density, extracts visual/collision meshes, and writes the Xacro files.

    ### Export Formats

    The tool supports two primary export formats via the `--format` option:

    - **`xacro` (Default)**: Generates a monolithic Xacro file. All links and joints are defined in a single file (or a small set of files), similar to traditional exporters.
    - **`xacro_module`**: Generates a **modular, hierarchical** Xacro description that mirrors your Onshape assembly structure.

    #### Benefits of Modular Export (`xacro_module`)

    - **Mirror CAD Hierarchy**: Each subassembly in Onshape is exported as a standalone Xacro macro in its own directory.
    - **Reusability**: If you use the same subassembly multiple times in Onshape (e.g., four identical legs), it is defined once and instantiated multiple times in Xacro.
    - **Clean Configuration**: Joint limits and inertial properties are stored in separate YAML files within each module's `config/` directory and loaded dynamically.
    - **Isolated Meshes**: Each module keeps its own meshes in a local `meshes/` folder, making the file structure much easier to navigate.

    Example modular structure:
    ```text
    output/
    └── urdf/
        ├── robot.urdf.xacro        # Main entry point
        ├── robot.xacro             # Root assembly module
        ├── leg/
        │   ├── leg.xacro           # Leg subassembly macro
        │   ├── config/             # Joint limits and inertials for leg
        │   └── meshes/             # Meshes used by leg
        └── arm/
            ├── arm.xacro
            └── ...
    ```

5. **BOM Export** (Optional):
    For inertial calculation, you can provide a Bill of Materials (BOM) CSV to specify material densities and mass for different parts:

    ```bash
    onshape2xacro export <local_dir> --output <final_xacro_dir> --bom <path_to_bom.csv>
    ```

    BOM can be exported from onshape's assembly page, make sure to include `Name`, `Material`, and `Mass` columns into the bill and make sure their values are presented:
    ![BOM Example](./assets/bom_example.png)

    The inertia calculation assumes that the part's mass is uniformly distributed (which is true for metals but not the case for 3D-printed parts).

    To debug the inertial calculation, inspect the generated `inertia_debug.md` file in the output directory and compare with [calculated values](https://cad.onshape.com/help/Content/massprops-asmb.htm?cshid=massprops_assembly) from onshape.

### Other Export Options

- `--visual-mesh-format {obj,dae,glb,stl}`: Choose the format for visual meshes (default: `obj`).
- `--collision-method {fast,coacd}`:
    - `fast`: Uses the simplified visual mesh as collision geometry.
    - `coacd`: Uses Approximate Convex Decomposition (CoACD) to generate high-quality convex collision hulls.
- `--max-depth <int>`: Limit how deep the tool traverses into subassemblies (default: 5).

## Limitation

### Requires zeroing robot pose before export

**Why is manual input required?** To correctly align the URDF joints with the exported STEP geometry, the tool requires the specific values of every mate at the time of export. However, the current Onshape API ([getMateValues](https://cad.onshape.com/glassworks/explorer/#/Assembly/getMateValues)) does not support recursive retrieval and only returns data for the root assembly. Consequently, you must manually specify the mate values for any joints located inside sub-assemblies to ensure the kinematic chain matches the link meshes.

### Limited type of mates supported

Currently only revolute mates supported, PRs are welcome!

## Acknowledgements

- **[onshape-robotics-toolkit](https://github.com/onshape-robotics/onshape-robotics-toolkit)**: This project directly utilizes the toolkit for CAD communication and kinematic graph construction.
- **[onshape2robot](https://github.com/Rhoban/onshape2robot)**: We take great inspiration from the pioneering work of the `onshape2robot` project.
