Turtlelib Library
A library for handling transformations in SE(2) and other turtlebot-related math.

# Components
- geometry2d - Handles 2D geometry primitives
- se2d - Handles 2D rigid body transformations
- frame_main - Perform some rigid body computations based on user input

# Conceptual Questions
1. If you needed to be able to ~normalize~ Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):
   - Propose three different designs for implementing the ~normalize~ functionality
      1) Add a normalize function to geometry2d
      2) In the main script, implement the calculation each time it is needed
      3) Overload the / operator to perform the calculation

   - Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.
      1) **Pro:** The function is easy to implement and can be used in other scripts.
         **Con:** The function is not very efficient and requires a lot of memory.
      2) **Pro:** The function is very efficient and does not require much memory.
         **Con:** The function is not reusable and must be implemented in each script that uses it.
      3) **Pro:** The function is very efficient and does not require much memory.
         **Con:** The function is not reusable and must be implemented in each script that uses it.
             I do not understand the pro's and cons of option 3. Operator overloading is effectively just a fancy syntax for naming a function.
             E.g., instead ofcalling it normalize it would be called operator/() and could effectively be used as a normalize function.
             However, the / operator is a binary operator not a unary operator so this actually could not be used for the normalize function.

   - Which of the methods would you implement and why?
      - I would implement method 1 because it is the easiest to implement and is reusable. I however did implement method 2 for frame_main.cpp because it was the most efficient and I did not need to reuse it. However, if I were to reuse it, I would implement method 1.

2. What is the difference between a class and a struct in C++?
   - A class and a struct in C++ are user-defined data types that can contain variables, functions, and other data types. The difference is that a struct defaults to public access while a class defaults to private access.

3. Why is Vector2D a struct and Transform2D a Class (refer to at least 2 specific C++ core guidelines in your answer)?
   - Vector2D is a struct because it is a simple data type that does not require any functions. Transform2D is a class because it is a more complex data type that requires functions. C.8 explains why it is important to use a class rather than a struct if any member is non-public due to readibility of making it clear that something is hidden. For ease of comprehension, a cass will alert programmers for the need for an invariant so use of a class would be more appropriate (C.2). If data members can be indpendently varied, a struct is more appropriate (C.2).

4. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?
   - Some of the constructors in Transform2D are explicit because they are single argument constructors. C.46 states that single argument constructors should be explicit in order to avoid unintentional type conversions.

5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?
   - Refer to [[https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con-constants-and-immutability][C++ Core Guidelines (Constants and Immutability)]] in your answer
      - Transform2D::inv() is declared const because it does not change the object. Transform2D::operator*=() is not declared const because it changes the object. Con.2 states that functions that do *not* change the object should be declared const. 
