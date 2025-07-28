class Note
{
    NoteId id;
    float radius;
    float x, y;
    float dx, dy;

    Note(NoteId id, float radius, float x, float y, float dx, float dy)
    {
        this.id = id;
        this.radius = radius;
        this.x = x;
        this.y = y;
        this.dx = dx;
        this.dy = dy;
    }

    void move(float dtMs)
    {
        x += dx * dtMs;
        y += dy * dtMs;
    }

    void draw()
    {
        stroke(30);
        strokeWeight(1);

        switch (id)
        {
            case Dong:
                fill(255, 0, 0);
                break;
            case Ka:
                fill(0, 255, 255);
                break;
        }
        ellipse(x, y, 2 * radius, 2 * radius);

        noStroke();
    }
}