namespace RazorTerm.Utils;

public static class StringUtils
{
    public static bool IsDebugMessage(this string str)
    {
        return str?.StartsWith("(D) ") == true;
    }

    public static bool IsErrorMessage(this string str)
    {
        return str?.StartsWith("(E) ") == true;
    }

    public static bool IsFlashProgressMessage(this string str)
    {
        return str?.Contains("Wrote and verified") == true;
    }
}