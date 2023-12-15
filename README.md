# Generic RCSP
A rust library for solving the resource constrained shortest path problem using a dynamic programming labeling algorithm.
Was developed for solving pricing problems for branch-and-price.

## Usage:

Implement the trait ``UserProblem``. Define the label data, resource extension function, and dominance function.

```rust
/// Main trait to implement that holds the user implementation of the problem.
///
/// Generics to implement:
/// - LabelMeta : Data to be stored in labels
/// - NodeWeight: Node attributes
/// - EdgeWeight: Edge attributes
/// - BranchFilter: Constraints made available during label extension
///
pub trait UserProblem<LabelMeta: Meta,  NodeWeight,EdgeWeight : UserEdgeWeight, BranchFilter>
{
    /// build your problem graph and return it
    ///
    /// is called once during initialization of solver and then stored
    fn create_graph(&mut self) -> ProblemGraph<NodeWeight, EdgeWeight>;

    /// Dominance function called in labeling algorithm.
    /// return true if label_a domiates label_b
    fn is_dominating(label_a: &Label<LabelMeta>, label_b: &Label<LabelMeta>, active_filters : &[BranchFilter]) -> bool;

    /// Label extension function
    ///
    /// Receives previous label, edge, node weights, branching filters, and reference to the sink node
    /// Return None if extension infeasible, otherwise return new Label.
    fn extend_label<'arena>(
        &self,
        existing_label : &'arena Label<'arena, LabelMeta>,
        edge : &EdgeReference<EdgeWeight>,
        source : &NodeWeight,
        target : &NodeWeight,
        active_filters : &[BranchFilter],
        sink : NodeIndex
    ) -> Option<Label<'arena,LabelMeta>>;

}
```

## License 
Copyright (C) 2023 Gregor Godbersen, [Gerhard Hiermann ](https://github.com/ghiermann)

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

