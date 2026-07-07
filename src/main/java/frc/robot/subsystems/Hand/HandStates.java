// Raymond: lowercase package.
package frc.robot.subsystems.hand;


import frc.robot.util.IState; // Raymond: blank line between the import and the enum decl.
public enum HandStates implements IState {
    DISABLED,
    IDLE,
    GRIPPING_CORAL,
    GRIPPING_ALGAE,
    RELEASING
}